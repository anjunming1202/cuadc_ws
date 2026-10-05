#!/usr/bin/env python3
"""Serve DroneLink video + telemetry on http://127.0.0.1:<port>/.

Video: Python owns air-side UDP ports — clean 5600 (+5601 shared) and CV
preview 5606 — forwards RTP to localhost FFmpeg remux (MPEG-TS), fans out to
/stream.ts and /stream_cv.ts. JSON on :5601 (legacy) or :5701 → /telemetry.json.
"""

from __future__ import annotations

import argparse
import json
import queue
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

VIEWER_DIR = Path(__file__).resolve().parent
if str(VIEWER_DIR) not in sys.path:
    sys.path.insert(0, str(VIEWER_DIR))
from field_pose import AltitudeTrace, FieldPoseTracker
DEFAULT_SDP = VIEWER_DIR.parent / "DroneLink.sdp"  # docs / air-link reference
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
DEFAULT_TELEMETRY_PORT = 5701  # preferred once Pi unit is updated
DEFAULT_RECORD_DIR = VIEWER_DIR / "recordings"  # 录制落点(地面机本地,按需查看)
AIR_RTP_PORT = 5600
AIR_SHARED_PORT = 5601  # legacy telemetry JSON and/or RTCP from some stacks
AIR_CV_RTP_PORT = 5606  # CV preview downlink (Pi cv_preview_ctl 转发)
CV_PREVIEW_CTRL_PORT = 5611  # Pi cv_preview_ctl UDP; default off until viewer button
VIDEO_FORWARD_CTRL_PORT = 5612  # Pi video_forward_ctl UDP; default on at boot
DEFAULT_AIR_CTRL_HOSTS = "192.168.50.1,10.10.10.1"  # Wi-Fi AP + Ethernet
LOCAL_RTP_PORT = 15600  # FFmpeg loopback RTP (clean)
LOCAL_RTCP_PORT = 15601  # FFmpeg loopback RTCP (clean)
LOCAL_CV_RTP_PORT = 15610  # FFmpeg loopback RTP (CV)
LOCAL_CV_RTCP_PORT = 15611  # FFmpeg loopback RTCP (CV)
TELEMETRY_STALE_S = 2.0
CLIENT_QUEUE_MAX = 64  # ~ drop old TS if a slow browser falls behind

_telemetry_lock = threading.Lock()
_latest_telemetry: dict | None = None
_latest_telemetry_recv_mono: float = 0.0
_field_pose = FieldPoseTracker()  # 每次 GUIDED 上升沿锁 H；退出 GUIDED 钉回 H
_alt_trace = AltitudeTrace(window_s=30.0)  # 近 30s 高度，给折线图
_air_ctrl_hosts: list[str] = [
    h.strip() for h in DEFAULT_AIR_CTRL_HOSTS.split(",") if h.strip()
]


def query_air_ctrl(cmd: str, port: int, timeout_s: float = 2.0) -> dict:
    """Ask Pi UDP ctl (cv 5611 / video 5612) for on|off|status."""
    payload = json.dumps({"cmd": cmd}).encode("utf-8")
    last_err = "no ctrl host"
    for host in _air_ctrl_hosts:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(timeout_s)
        try:
            sock.sendto(payload, (host, port))
            data, _addr = sock.recvfrom(4096)
            obj = json.loads(data.decode("utf-8"))
            if isinstance(obj, dict):
                obj.setdefault("host", host)
                return obj
        except Exception as exc:  # noqa: BLE001  # 一主机失败换下一个
            last_err = f"{host}: {exc}"
        finally:
            sock.close()
    return {"ok": False, "running": False, "error": last_err}


def find_ffmpeg() -> str:
    path = shutil.which("ffmpeg")
    if path:
        return path
    raise SystemExit(
        "ffmpeg not found on PATH. Install with: winget install --id Gyan.FFmpeg -e"
    )


def _ingest_telemetry_payload(payload: bytes, addr: tuple[str, int]) -> bool:
    """Return True if payload was telemetry JSON."""
    global _latest_telemetry, _latest_telemetry_recv_mono
    try:
        obj = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return False
    if not isinstance(obj, dict):
        return False
    with _telemetry_lock:
        obj.update(_field_pose.update(obj))
        _alt_trace.update(obj)
        obj["alt_trace"] = _alt_trace.snapshot()
        _latest_telemetry = obj
        _latest_telemetry_recv_mono = time.monotonic()
    return True


def _ffmpeg_cmd(ffmpeg_bin: str, sdp_path: Path) -> list[str]:
    # Live RTP often starts mid-GOP; wait for SPS/PPS and allow RTCP.
    return [
        ffmpeg_bin,
        "-hide_banner",
        "-loglevel",
        "warning",
        "-fflags",
        "+genpts+discardcorrupt+nobuffer",
        "-flags",
        "low_delay",
        "-err_detect",
        "ignore_err",
        "-thread_queue_size",
        "1024",
        "-probesize",
        "5M",
        "-analyzeduration",
        "5M",
        "-protocol_whitelist",
        "file,udp,rtp,rtcp",
        "-i",
        str(sdp_path),
        "-an",
        "-c:v",
        "copy",
        "-bsf:v",
        "dump_extra",
        "-muxdelay",
        "0",
        "-muxpreload",
        "0",
        "-f",
        "mpegts",
        "pipe:1",
    ]


def write_loopback_sdp(
    path: Path,
    *,
    rtp_port: int,
    rtcp_port: int,
    session_name: str = "DroneLink H264 loopback",
) -> None:
    path.write_text(
        "\n".join(
            [
                "v=0",
                "o=- 0 0 IN IP4 127.0.0.1",
                f"s={session_name}",
                "c=IN IP4 127.0.0.1",
                "t=0 0",
                f"m=video {rtp_port} RTP/AVP 96",
                "a=rtpmap:96 H264/90000",
                "a=fmtp:96 packetization-mode=1",
                f"a=rtcp:{rtcp_port} IN IP4 127.0.0.1",
                "",
            ]
        ),
        encoding="ascii",
    )


class AirLinkProxy:
    """Own air Wi-Fi UDP ports; demux telemetry JSON vs RTP/RTCP for FFmpeg."""

    def __init__(self, air_bind: str = "0.0.0.0") -> None:
        self._air_bind = air_bind
        self._stop = threading.Event()
        self._fwd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._threads: list[threading.Thread] = []

    def start(self) -> None:
        rtp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rtp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        rtp.bind((self._air_bind, AIR_RTP_PORT))
        shared = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        shared.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        shared.bind((self._air_bind, AIR_SHARED_PORT))
        sys.stderr.write(
            f"AirLink proxy listening on {self._air_bind}:{AIR_RTP_PORT} (RTP) "
            f"and :{AIR_SHARED_PORT} (telemetry JSON / RTCP)\n"
        )
        self._threads = [
            threading.Thread(
                target=self._rtp_loop, args=(rtp,), name="air-rtp", daemon=True
            ),
            threading.Thread(
                target=self._shared_loop, args=(shared,), name="air-shared", daemon=True
            ),
        ]
        for t in self._threads:
            t.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            self._fwd.close()
        except OSError:
            pass

    def _rtp_loop(self, sock: socket.socket) -> None:
        sock.settimeout(0.5)
        while not self._stop.is_set():
            try:
                payload, _addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                self._fwd.sendto(payload, ("127.0.0.1", LOCAL_RTP_PORT))
            except OSError:
                pass
        sock.close()

    def _shared_loop(self, sock: socket.socket) -> None:
        sock.settimeout(0.5)
        while not self._stop.is_set():
            try:
                payload, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if _ingest_telemetry_payload(payload, addr):
                continue
            # Non-JSON: treat as RTCP (or other) and forward to loopback FFmpeg.
            try:
                self._fwd.sendto(payload, ("127.0.0.1", LOCAL_RTCP_PORT))
            except OSError:
                pass
        sock.close()


class SimpleRtpProxy:
    """Own one air UDP port; forward datagrams to a localhost RTP port (no demux)."""

    def __init__(
        self,
        air_port: int,
        local_rtp_port: int,
        *,
        air_bind: str = "0.0.0.0",
        name: str = "rtp",
    ) -> None:
        self._air_bind = air_bind
        self._air_port = air_port
        self._local_rtp_port = local_rtp_port
        self._name = name
        self._stop = threading.Event()
        self._fwd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._air_bind, self._air_port))
        sys.stderr.write(
            f"{self._name} proxy listening on {self._air_bind}:{self._air_port} "
            f"-> 127.0.0.1:{self._local_rtp_port}\n"
        )
        self._thread = threading.Thread(
            target=self._loop, args=(sock,), name=f"{self._name}-rtp", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            self._fwd.close()
        except OSError:
            pass

    def _loop(self, sock: socket.socket) -> None:
        sock.settimeout(0.5)
        while not self._stop.is_set():
            try:
                payload, _addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                self._fwd.sendto(payload, ("127.0.0.1", self._local_rtp_port))
            except OSError:
                pass
        sock.close()


class RemuxHub:
    """Single FFmpeg remuxes loopback RTP; HTTP clients subscribe to chunks."""

    def __init__(self, ffmpeg_bin: str, sdp_path: Path) -> None:
        self._ffmpeg_bin = ffmpeg_bin
        self._sdp_path = sdp_path
        self._clients: list[queue.Queue[bytes | None]] = []
        self._clients_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="remux-hub", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._broadcast(None)

    def subscribe(self) -> queue.Queue[bytes | None]:
        q: queue.Queue[bytes | None] = queue.Queue(maxsize=CLIENT_QUEUE_MAX)
        with self._clients_lock:
            self._clients.append(q)
        return q

    def unsubscribe(self, q: queue.Queue[bytes | None]) -> None:
        with self._clients_lock:
            if q in self._clients:
                self._clients.remove(q)

    def _broadcast(self, chunk: bytes | None) -> None:
        with self._clients_lock:
            targets = list(self._clients)
        for q in targets:
            try:
                q.put_nowait(chunk)
            except queue.Full:
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    q.put_nowait(chunk)
                except queue.Full:
                    pass

    def _run(self) -> None:
        backoff = 1.0
        while not self._stop.is_set():
            cmd = _ffmpeg_cmd(self._ffmpeg_bin, self._sdp_path)
            sys.stderr.write("Starting shared FFmpeg remux (loopback SDP)…\n")
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    bufsize=0,
                )
            except OSError as exc:
                sys.stderr.write(f"failed to start ffmpeg: {exc}\n")
                time.sleep(backoff)
                backoff = min(8.0, backoff * 1.5)
                continue

            def _drain_stderr() -> None:
                assert proc.stderr is not None
                for line in iter(proc.stderr.readline, b""):
                    sys.stderr.write(line.decode("utf-8", errors="replace"))

            threading.Thread(target=_drain_stderr, daemon=True).start()
            assert proc.stdout is not None
            got_data = False
            try:
                while not self._stop.is_set():
                    chunk = proc.stdout.read(188 * 64)
                    if not chunk:
                        break
                    got_data = True
                    backoff = 1.0
                    self._broadcast(chunk)
            finally:
                if proc.poll() is None:
                    proc.kill()
                    try:
                        proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                time.sleep(0.8)

            if self._stop.is_set():
                break
            why = "no RTP yet / ffmpeg exited" if not got_data else "ffmpeg exited"
            sys.stderr.write(
                f"Shared FFmpeg stopped ({why}); retry in {backoff:.1f}s. "
                "If you see bind -10048, run: taskkill /IM ffmpeg.exe /F\n"
            )
            time.sleep(backoff)
            backoff = min(8.0, backoff * 1.5)


class Recorder:
    """录制:给每个 RemuxHub 挂一个订阅者,把 MPEG-TS 分片写文件 + 一份 .jsonl 遥测。
    默认同时录**干净图传** `<name>.ts` 与 **CV 预览** `<name>_cv.ts`(带检测框),
    外加每秒一条 `<name>.jsonl` 遥测。只是多几个订阅者,不碰视频主路 → 和任何
    测试/任务代码解耦;按需 start/stop,现场任何时候都能开。"""

    def __init__(self, hubs: dict[str, RemuxHub], record_dir: Path) -> None:
        self._hubs = hubs                       # {文件名后缀: hub},如 {"": 干净, "_cv": CV}
        self._dir = Path(record_dir)
        self._lock = threading.Lock()
        self._bytes_lock = threading.Lock()
        self._active = False
        self._threads: list[threading.Thread] = []
        self._name: str | None = None
        self._telem_path: Path | None = None
        self._started_mono = 0.0
        self._bytes = 0

    def start(self, label: str | None = None) -> dict:
        with self._lock:
            if self._active:
                return {"ok": False, "error": "already recording", **self._status_unlocked()}
            self._dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self._name = f"{stamp}_{label}" if label else stamp
            self._telem_path = self._dir / f"{self._name}.jsonl"
            self._active = True
            self._bytes = 0
            self._started_mono = time.monotonic()
            self._threads = []
            for suffix, hub in self._hubs.items():
                path = self._dir / f"{self._name}{suffix}.ts"
                t = threading.Thread(target=self._run_video, args=(hub, path),
                                     name=f"rec{suffix or '-clean'}", daemon=True)
                t.start()
                self._threads.append(t)
            tt = threading.Thread(target=self._run_telem, name="rec-telem", daemon=True)
            tt.start()
            self._threads.append(tt)
            sys.stderr.write(f"[rec] start -> {self._dir}/{self._name}{{.ts,_cv.ts,.jsonl}}\n")
            return {"ok": True, **self._status_unlocked()}

    def stop(self) -> dict:
        with self._lock:
            if not self._active:
                return {"ok": False, "error": "not recording"}
            st = self._status_unlocked()   # 先取快照(还 active,秒数才对)
            self._active = False
            threads = list(self._threads)
        for t in threads:
            t.join(timeout=3)
        st["recording"] = False
        sys.stderr.write(f"[rec] stop  ({st['name']}, {st['seconds']}s, {st['bytes']} B)\n")
        return {"ok": True, "stopped": True, **st}

    def status(self) -> dict:
        with self._lock:
            return self._status_unlocked()

    def _status_unlocked(self) -> dict:
        with self._bytes_lock:
            nbytes = self._bytes
        return {
            "recording": self._active,
            "name": self._name if self._active else None,
            "streams": list(self._hubs.keys()),
            "seconds": round(time.monotonic() - self._started_mono, 1) if self._active else 0.0,
            "bytes": nbytes,
            "dir": str(self._dir),
        }

    def _run_video(self, hub: RemuxHub, path: Path) -> None:
        q = hub.subscribe()
        try:
            with open(path, "wb") as vf:
                while self._active:
                    try:
                        chunk = q.get(timeout=0.5)
                    except queue.Empty:
                        continue
                    if chunk is None:        # hub 停了
                        break
                    vf.write(chunk)
                    with self._bytes_lock:
                        self._bytes += len(chunk)
                vf.flush()
        finally:
            hub.unsubscribe(q)

    def _run_telem(self) -> None:
        last = 0.0
        with open(self._telem_path, "w", encoding="utf-8") as tf:
            while self._active:
                now = time.monotonic()
                if now - last >= 1.0:        # 每秒一条,和视频同名对齐
                    last = now
                    tf.write(json.dumps({"t": time.time(), **_telemetry_snapshot()},
                                        ensure_ascii=False) + "\n")
                    tf.flush()
                time.sleep(0.2)


_hub: RemuxHub | None = None
_cv_hub: RemuxHub | None = None
_proxy: AirLinkProxy | None = None
_cv_proxy: SimpleRtpProxy | None = None
_recorder: Recorder | None = None


def _telemetry_snapshot() -> dict:
    with _telemetry_lock:
        age = (
            time.monotonic() - _latest_telemetry_recv_mono
            if _latest_telemetry is not None
            else None
        )
        fresh = age is not None and age <= TELEMETRY_STALE_S
        data = _latest_telemetry
        if data is None:
            data = {
                "field_x_m": 0.0,
                "field_y_m": 0.0,
                "field_yaw_deg": 0.0,
                "field_airborne": False,
                "field_src": "pre-guided",
                "alt_trace": _alt_trace.snapshot(),
            }
        return {
            "ok": bool(fresh),
            "age_s": None if age is None else round(age, 3),
            "data": data,
            "stale": bool(_latest_telemetry is not None and not fresh),
        }


def start_telemetry_listener(bind_host: str, port: int) -> None:
    def _loop() -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((bind_host, port))
        sys.stderr.write(f"Telemetry UDP listening on {bind_host}:{port}\n")
        pkt_count = 0
        last_log = 0.0
        while True:
            try:
                payload, addr = sock.recvfrom(65535)
            except OSError as exc:
                sys.stderr.write(f"telemetry socket error: {exc}\n")
                break
            if not _ingest_telemetry_payload(payload, addr):
                sys.stderr.write(f"telemetry bad packet ignored from {addr}\n")
                continue
            pkt_count += 1
            now = time.monotonic()
            if pkt_count == 1 or now - last_log > 10.0:
                last_log = now
                with _telemetry_lock:
                    obj = _latest_telemetry or {}
                sys.stderr.write(
                    f"telemetry ok from {addr[0]}:{addr[1]} "
                    f"pkts={pkt_count} mode={obj.get('mode')} alt={obj.get('alt_m')}\n"
                )

    threading.Thread(target=_loop, name="telemetry-udp", daemon=True).start()


class ViewerHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args) -> None:
        sys.stderr.write("[%s] %s\n" % (self.log_date_time_string(), fmt % args))

    def _send_bytes(self, data: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, obj: dict) -> None:
        self._send_bytes(json.dumps(obj, ensure_ascii=False).encode("utf-8"),
                         "application/json; charset=utf-8")

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            body = (VIEWER_DIR / "index.html").read_bytes()
            self._send_bytes(body, "text/html; charset=utf-8")
            return
        if path == "/mpegts.js":
            body = (VIEWER_DIR / "mpegts.js").read_bytes()
            self._send_bytes(body, "application/javascript; charset=utf-8")
            return
        if path == "/field_map.js":
            body = (VIEWER_DIR / "field_map.js").read_bytes()
            self._send_bytes(body, "application/javascript; charset=utf-8")
            return
        if path == "/telemetry.json":
            body = json.dumps(_telemetry_snapshot(), ensure_ascii=False).encode("utf-8")
            self._send_bytes(body, "application/json; charset=utf-8")
            return
        if path == "/stream.ts":
            self._stream_ts(_hub, "clean remux hub")
            return
        if path == "/stream_cv.ts":
            self._stream_ts(_cv_hub, "CV remux hub")
            return
        if path == "/record/status":
            self._send_json(_recorder.status() if _recorder else {"recording": False})
            return
        if path == "/cv-preview/status":
            self._send_json(query_air_ctrl("status", CV_PREVIEW_CTRL_PORT))
            return
        if path == "/video-forward/status":
            self._send_json(query_air_ctrl("status", VIDEO_FORWARD_CTRL_PORT))
            return
        self.send_error(404)

    def do_POST(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path == "/cv-preview/start":
            self._send_json(query_air_ctrl("on", CV_PREVIEW_CTRL_PORT))
            return
        if path == "/cv-preview/stop":
            self._send_json(query_air_ctrl("off", CV_PREVIEW_CTRL_PORT))
            return
        if path == "/video-forward/start":
            self._send_json(query_air_ctrl("on", VIDEO_FORWARD_CTRL_PORT))
            return
        if path == "/video-forward/stop":
            self._send_json(query_air_ctrl("off", VIDEO_FORWARD_CTRL_PORT))
            return
        if _recorder is None:
            self.send_error(503, "recorder not ready")
            return
        if path == "/record/start":
            label = (parse_qs(urlparse(self.path).query).get("label") or [None])[0]
            self._send_json(_recorder.start(label))
            return
        if path == "/record/stop":
            self._send_json(_recorder.stop())
            return
        self.send_error(404)

    def _stream_ts(self, hub: RemuxHub | None, hub_label: str) -> None:
        if hub is None:
            self.send_error(503, f"{hub_label} not ready")
            return
        self.send_response(200)
        self.send_header("Content-Type", "video/mp2t")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        q = hub.subscribe()
        try:
            while True:
                chunk = q.get()
                if chunk is None:
                    break
                self.wfile.write(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            hub.unsubscribe(q)


def main() -> None:
    global _hub, _cv_hub, _proxy, _cv_proxy, _recorder, _air_ctrl_hosts

    parser = argparse.ArgumentParser(description="DroneLink localhost video viewer")
    parser.add_argument("--host", default=DEFAULT_HOST, help="HTTP bind address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="HTTP port")
    parser.add_argument("--telemetry-bind", default="0.0.0.0")
    parser.add_argument("--telemetry-port", type=int, default=DEFAULT_TELEMETRY_PORT)
    parser.add_argument(
        "--sdp",
        type=Path,
        default=DEFAULT_SDP,
        help="unused for remux (kept for CLI compat); air ports are proxied",
    )
    parser.add_argument("--record-dir", type=Path, default=DEFAULT_RECORD_DIR,
                        help="录制输出目录(默认 groundstation/recordings/)")
    parser.add_argument(
        "--air-ctrl-hosts",
        default=DEFAULT_AIR_CTRL_HOSTS,
        help="Pi UDP ctl hosts (cv 5611 + video 5612), comma-separated",
    )
    args = parser.parse_args()
    _air_ctrl_hosts = [h.strip() for h in args.air_ctrl_hosts.split(",") if h.strip()]

    ffmpeg_bin = find_ffmpeg()
    start_telemetry_listener(args.telemetry_bind, args.telemetry_port)

    _proxy = AirLinkProxy(air_bind="0.0.0.0")
    _proxy.start()
    _cv_proxy = SimpleRtpProxy(
        AIR_CV_RTP_PORT, LOCAL_CV_RTP_PORT, air_bind="0.0.0.0", name="CV"
    )
    _cv_proxy.start()

    tmp = Path(tempfile.gettempdir())
    loopback_sdp = tmp / "dronelink_loopback.sdp"
    write_loopback_sdp(
        loopback_sdp,
        rtp_port=LOCAL_RTP_PORT,
        rtcp_port=LOCAL_RTCP_PORT,
        session_name="DroneLink clean H264",
    )
    cv_sdp = tmp / "dronelink_cv_loopback.sdp"
    write_loopback_sdp(
        cv_sdp,
        rtp_port=LOCAL_CV_RTP_PORT,
        rtcp_port=LOCAL_CV_RTCP_PORT,
        session_name="DroneLink CV H264",
    )

    _hub = RemuxHub(ffmpeg_bin, loopback_sdp)
    _hub.start()
    _cv_hub = RemuxHub(ffmpeg_bin, cv_sdp)
    _cv_hub.start()
    _recorder = Recorder({"": _hub, "_cv": _cv_hub}, args.record_dir)

    server = ThreadingHTTPServer((args.host, args.port), ViewerHandler)
    print(f"DroneLink viewer: http://{args.host}:{args.port}/")
    print(f"FFmpeg: {ffmpeg_bin}")
    print(f"Clean RTP/shared: 0.0.0.0:{AIR_RTP_PORT}/:{AIR_SHARED_PORT} -> /stream.ts")
    print(f"CV RTP: 0.0.0.0:{AIR_CV_RTP_PORT} -> /stream_cv.ts")
    print(f"Telemetry UDP: {args.telemetry_bind}:{args.telemetry_port} (+ JSON on :{AIR_SHARED_PORT})")
    print(f"Recordings: {args.record_dir}  (POST /record/start|stop, GET /record/status)")
    print(
        f"Air ctrl hosts {_air_ctrl_hosts}: "
        f"CV :{CV_PREVIEW_CTRL_PORT} (POST /cv-preview/start|stop) · "
        f"video :{VIDEO_FORWARD_CTRL_PORT} (POST /video-forward/start|stop)"
    )
    print("Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if _recorder is not None and _recorder.status().get("recording"):
            _recorder.stop()
        if _hub is not None:
            _hub.stop()
        if _cv_hub is not None:
            _cv_hub.stop()
        if _proxy is not None:
            _proxy.stop()
        if _cv_proxy is not None:
            _cv_proxy.stop()
        server.server_close()


if __name__ == "__main__":
    main()
