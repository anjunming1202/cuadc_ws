#!/usr/bin/env python3
"""Serve DroneLink video + telemetry on http://127.0.0.1:<port>/.

Video: Python owns the air-side UDP ports (5600 RTP / 5601 shared), forwards
RTP+RTCP to localhost for one shared FFmpeg remux (MPEG-TS), fans out to
/stream.ts clients. JSON datagrams that arrive on :5601 (legacy Pi) or :5701
are exposed as /telemetry.json for the HUD.
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
DEFAULT_SDP = VIEWER_DIR.parent / "DroneLink.sdp"  # docs / air-link reference
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
DEFAULT_TELEMETRY_PORT = 5701  # preferred once Pi unit is updated
DEFAULT_RECORD_DIR = VIEWER_DIR / "recordings"  # 录制落点(地面机本地,按需查看)
AIR_RTP_PORT = 5600
AIR_SHARED_PORT = 5601  # legacy telemetry JSON and/or RTCP from some stacks
LOCAL_RTP_PORT = 15600  # FFmpeg loopback RTP
LOCAL_RTCP_PORT = 15601  # FFmpeg loopback RTCP
TELEMETRY_STALE_S = 2.0
CLIENT_QUEUE_MAX = 64  # ~ drop old TS if a slow browser falls behind

_telemetry_lock = threading.Lock()
_latest_telemetry: dict | None = None
_latest_telemetry_recv_mono: float = 0.0


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


def write_loopback_sdp(path: Path) -> None:
    path.write_text(
        "\n".join(
            [
                "v=0",
                "o=- 0 0 IN IP4 127.0.0.1",
                "s=DroneLink H264 loopback",
                "c=IN IP4 127.0.0.1",
                "t=0 0",
                f"m=video {LOCAL_RTP_PORT} RTP/AVP 96",
                "a=rtpmap:96 H264/90000",
                "a=fmtp:96 packetization-mode=1",
                f"a=rtcp:{LOCAL_RTCP_PORT} IN IP4 127.0.0.1",
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
    """录制:作为 RemuxHub 的一个订阅者,把 MPEG-TS 分片写 <ts>.ts + 同名 <ts>.jsonl 遥测。
    只是多一个订阅者,不碰视频主路 → 和任何测试/任务代码解耦;按需 start/stop。"""

    def __init__(self, hub: RemuxHub, record_dir: Path) -> None:
        self._hub = hub
        self._dir = Path(record_dir)
        self._lock = threading.Lock()
        self._active = False
        self._thread: threading.Thread | None = None
        self._path: Path | None = None
        self._telem_path: Path | None = None
        self._started_mono = 0.0
        self._bytes = 0

    def start(self, label: str | None = None) -> dict:
        with self._lock:
            if self._active:
                return {"ok": False, "error": "already recording", **self._status_unlocked()}
            self._dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            name = f"{stamp}_{label}" if label else stamp
            self._path = self._dir / f"{name}.ts"
            self._telem_path = self._dir / f"{name}.jsonl"
            self._active = True
            self._bytes = 0
            self._started_mono = time.monotonic()
            self._thread = threading.Thread(target=self._run, name="recorder", daemon=True)
            self._thread.start()
            sys.stderr.write(f"[rec] start -> {self._path}\n")
            return {"ok": True, **self._status_unlocked()}

    def stop(self) -> dict:
        with self._lock:
            if not self._active:
                return {"ok": False, "error": "not recording"}
            st = self._status_unlocked()   # 先取快照(还 active,秒数才对)
            self._active = False
            thread = self._thread
        if thread:
            thread.join(timeout=3)
        st["recording"] = False
        sys.stderr.write(f"[rec] stop  ({st['file']}, {st['seconds']}s, {st['bytes']} B)\n")
        return {"ok": True, "stopped": True, **st}

    def status(self) -> dict:
        with self._lock:
            return self._status_unlocked()

    def _status_unlocked(self) -> dict:
        return {
            "recording": self._active,
            "file": self._path.name if self._path else None,
            "seconds": round(time.monotonic() - self._started_mono, 1) if self._active else 0.0,
            "bytes": self._bytes,
            "dir": str(self._dir),
        }

    def _run(self) -> None:
        q = self._hub.subscribe()
        last_telem = 0.0
        try:
            with open(self._path, "wb") as vf, open(self._telem_path, "w", encoding="utf-8") as tf:
                while self._active:
                    try:
                        chunk = q.get(timeout=0.5)
                    except queue.Empty:
                        chunk = b""          # 无视频也继续循环(采遥测)
                    if chunk is None:        # hub 停了
                        break
                    if chunk:
                        vf.write(chunk)
                        self._bytes += len(chunk)
                    now = time.monotonic()
                    if now - last_telem >= 1.0:  # 每秒一条遥测,和视频同名对齐
                        last_telem = now
                        tf.write(json.dumps({"t": time.time(), **_telemetry_snapshot()},
                                            ensure_ascii=False) + "\n")
                        tf.flush()
                vf.flush()
        finally:
            self._hub.unsubscribe(q)


_hub: RemuxHub | None = None
_proxy: AirLinkProxy | None = None
_recorder: Recorder | None = None


def _telemetry_snapshot() -> dict:
    with _telemetry_lock:
        age = (
            time.monotonic() - _latest_telemetry_recv_mono
            if _latest_telemetry is not None
            else None
        )
        fresh = age is not None and age <= TELEMETRY_STALE_S
        return {
            "ok": bool(fresh),
            "age_s": None if age is None else round(age, 3),
            "data": _latest_telemetry,
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
        if path == "/telemetry.json":
            body = json.dumps(_telemetry_snapshot(), ensure_ascii=False).encode("utf-8")
            self._send_bytes(body, "application/json; charset=utf-8")
            return
        if path == "/stream.ts":
            self._stream_ts()
            return
        if path == "/record/status":
            self._send_json(_recorder.status() if _recorder else {"recording": False})
            return
        self.send_error(404)

    def do_POST(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
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

    def _stream_ts(self) -> None:
        global _hub
        if _hub is None:
            self.send_error(503, "remux hub not ready")
            return
        self.send_response(200)
        self.send_header("Content-Type", "video/mp2t")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        q = _hub.subscribe()
        try:
            while True:
                chunk = q.get()
                if chunk is None:
                    break
                self.wfile.write(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            _hub.unsubscribe(q)


def main() -> None:
    global _hub, _proxy, _recorder

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
    args = parser.parse_args()

    ffmpeg_bin = find_ffmpeg()
    start_telemetry_listener(args.telemetry_bind, args.telemetry_port)

    _proxy = AirLinkProxy(air_bind="0.0.0.0")
    _proxy.start()

    loopback_sdp = Path(tempfile.gettempdir()) / "dronelink_loopback.sdp"
    write_loopback_sdp(loopback_sdp)

    _hub = RemuxHub(ffmpeg_bin, loopback_sdp)
    _hub.start()
    _recorder = Recorder(_hub, args.record_dir)

    server = ThreadingHTTPServer((args.host, args.port), ViewerHandler)
    print(f"DroneLink viewer: http://{args.host}:{args.port}/")
    print(f"FFmpeg: {ffmpeg_bin}")
    print(f"Air RTP/shared: 0.0.0.0:{AIR_RTP_PORT}/:{AIR_SHARED_PORT} -> loopback")
    print(f"Telemetry UDP: {args.telemetry_bind}:{args.telemetry_port} (+ JSON on :{AIR_SHARED_PORT})")
    print(f"Recordings: {args.record_dir}  (POST /record/start|stop, GET /record/status)")
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
        if _proxy is not None:
            _proxy.stop()
        server.server_close()


if __name__ == "__main__":
    main()
