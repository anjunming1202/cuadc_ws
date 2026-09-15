#!/usr/bin/env python3
"""Serve DroneLink video + telemetry on http://127.0.0.1:<port>/.

Video: one shared FFmpeg process remuxes RTP (DroneLink.sdp) to MPEG-TS and
fans out to all /stream.ts clients (avoids Windows UDP :5600 bind races).
Telemetry: UDP JSON on TELEMETRY_PORT -> /telemetry.json (polled by the page)
"""

from __future__ import annotations

import argparse
import json
import queue
import shutil
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

VIEWER_DIR = Path(__file__).resolve().parent
DEFAULT_SDP = VIEWER_DIR.parent / "DroneLink.sdp"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
DEFAULT_TELEMETRY_PORT = 5701  # not 5601: FFmpeg RTP on :5600 also binds RTCP on :5601
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


def _ffmpeg_cmd(ffmpeg_bin: str, sdp_path: Path) -> list[str]:
    # Live RTP often starts mid-GOP; wait for SPS/PPS (up to ~2s) and allow RTCP.
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


class RemuxHub:
    """Single FFmpeg owns UDP :5600; HTTP clients only subscribe to chunks."""

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
            sys.stderr.write("Starting shared FFmpeg remux (binds SDP UDP port once)…\n")
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
                # Windows needs a beat to release UDP :5600
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


_hub: RemuxHub | None = None


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
        global _latest_telemetry, _latest_telemetry_recv_mono
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((bind_host, port))
        sys.stderr.write(f"Telemetry UDP listening on {bind_host}:{port}\n")
        while True:
            try:
                payload, _addr = sock.recvfrom(65535)
            except OSError as exc:
                sys.stderr.write(f"telemetry socket error: {exc}\n")
                break
            try:
                obj = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                sys.stderr.write(f"telemetry bad packet ignored: {exc}\n")
                continue
            if not isinstance(obj, dict):
                continue
            with _telemetry_lock:
                _latest_telemetry = obj
                _latest_telemetry_recv_mono = time.monotonic()

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

    def _send_file(self, path: Path, content_type: str) -> None:
        self._send_bytes(path.read_bytes(), content_type)

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._send_file(VIEWER_DIR / "index.html", "text/html; charset=utf-8")
            return
        if path == "/mpegts.js":
            self._send_file(VIEWER_DIR / "mpegts.js", "application/javascript; charset=utf-8")
            return
        if path == "/telemetry.json":
            body = json.dumps(_telemetry_snapshot(), ensure_ascii=False).encode("utf-8")
            self._send_bytes(body, "application/json; charset=utf-8")
            return
        if path == "/stream.ts":
            self._stream_mpegts()
            return
        self.send_error(404, "Not found")

    def _stream_mpegts(self) -> None:
        if _hub is None:
            self.send_error(503, "remux hub not ready")
            return

        self.send_response(200)
        self.send_header("Content-Type", "video/mp2t")
        self.send_header("Cache-Control", "no-cache, no-store")
        self.send_header("Connection", "close")
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
    global _hub

    parser = argparse.ArgumentParser(description="DroneLink localhost video viewer")
    parser.add_argument("--host", default=DEFAULT_HOST, help="HTTP bind address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="HTTP port")
    parser.add_argument("--telemetry-bind", default="0.0.0.0")
    parser.add_argument("--telemetry-port", type=int, default=DEFAULT_TELEMETRY_PORT)
    parser.add_argument("--sdp", type=Path, default=DEFAULT_SDP)
    args = parser.parse_args()

    sdp = args.sdp.resolve()
    if not sdp.is_file():
        raise SystemExit(f"SDP file not found: {sdp}")

    ffmpeg_bin = find_ffmpeg()
    start_telemetry_listener(args.telemetry_bind, args.telemetry_port)

    _hub = RemuxHub(ffmpeg_bin, sdp)
    _hub.start()

    server = ThreadingHTTPServer((args.host, args.port), ViewerHandler)
    print(f"DroneLink viewer: http://{args.host}:{args.port}/")
    print(f"SDP: {sdp}")
    print(f"FFmpeg: {ffmpeg_bin}")
    print("Video RTP: address/port in SDP (default 192.168.50.2:5600)")
    print(f"Telemetry UDP: {args.telemetry_bind}:{args.telemetry_port}")
    print("Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if _hub is not None:
            _hub.stop()
        server.server_close()


if __name__ == "__main__":
    main()
