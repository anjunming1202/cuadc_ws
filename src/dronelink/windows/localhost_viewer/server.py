#!/usr/bin/env python3
"""Serve DroneLink H.264 RTP on http://127.0.0.1:<port>/ via MPEG-TS remux.

Receives the existing UDP RTP stream described by DroneLink.sdp, remuxes with
FFmpeg (copy, no re-encode), and serves a localhost page that plays it with
mpegts.js. Telemetry overlay comes later.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

VIEWER_DIR = Path(__file__).resolve().parent
DEFAULT_SDP = VIEWER_DIR.parent / "DroneLink.sdp"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765

# Only one FFmpeg can bind the SDP UDP port; replace the previous reader on refresh.
_stream_lock = threading.Lock()
_active_ffmpeg: subprocess.Popen[bytes] | None = None


def find_ffmpeg() -> str:
    path = shutil.which("ffmpeg")
    if path:
        return path
    raise SystemExit(
        "ffmpeg not found on PATH. Install with: winget install --id Gyan.FFmpeg -e"
    )


def _stop_active_ffmpeg() -> None:
    global _active_ffmpeg
    proc = _active_ffmpeg
    _active_ffmpeg = None
    if proc is None:
        return
    if proc.poll() is None:
        proc.kill()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()


def _ffmpeg_cmd(ffmpeg_bin: str, sdp_path: Path) -> list[str]:
    # RTP/UDP over Wi-Fi drops packets even when the association stays up.
    # discardcorrupt + ignore_err keep remux alive across brief gaps; the
    # browser player still auto-reconnects if MSE gets a hard error.
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
        "512",
        "-probesize",
        "32",
        "-analyzeduration",
        "0",
        "-protocol_whitelist",
        "file,udp,rtp",
        "-i",
        str(sdp_path),
        "-an",
        "-c:v",
        "copy",
        "-muxdelay",
        "0",
        "-muxpreload",
        "0",
        "-f",
        "mpegts",
        "pipe:1",
    ]


class ViewerHandler(BaseHTTPRequestHandler):
    sdp_path: Path = DEFAULT_SDP
    ffmpeg_bin: str = "ffmpeg"

    def log_message(self, fmt: str, *args) -> None:
        sys.stderr.write("[%s] %s\n" % (self.log_date_time_string(), fmt % args))

    def _send_file(self, path: Path, content_type: str) -> None:
        data = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._send_file(VIEWER_DIR / "index.html", "text/html; charset=utf-8")
            return
        if path == "/mpegts.js":
            self._send_file(VIEWER_DIR / "mpegts.js", "application/javascript; charset=utf-8")
            return
        if path == "/stream.ts":
            self._stream_mpegts()
            return
        self.send_error(404, "Not found")

    def _stream_mpegts(self) -> None:
        global _active_ffmpeg

        if not self.sdp_path.is_file():
            self.send_error(500, f"SDP not found: {self.sdp_path}")
            return

        self.send_response(200)
        self.send_header("Content-Type", "video/mp2t")
        self.send_header("Cache-Control", "no-cache, no-store")
        self.send_header("Connection", "close")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        # Keep the HTTP response open across FFmpeg restarts so a brief RTP
        # glitch does not permanently end /stream.ts for this client.
        client_alive = True
        while client_alive:
            cmd = _ffmpeg_cmd(self.ffmpeg_bin, self.sdp_path)
            with _stream_lock:
                _stop_active_ffmpeg()
                try:
                    proc = subprocess.Popen(
                        cmd,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        bufsize=0,
                    )
                except OSError as exc:
                    sys.stderr.write(f"failed to start ffmpeg: {exc}\n")
                    break
                _active_ffmpeg = proc

            def _drain_stderr(p: subprocess.Popen[bytes] = proc) -> None:
                assert p.stderr is not None
                for line in iter(p.stderr.readline, b""):
                    sys.stderr.write(line.decode("utf-8", errors="replace"))

            threading.Thread(target=_drain_stderr, daemon=True).start()
            assert proc.stdout is not None
            try:
                while True:
                    chunk = proc.stdout.read(188 * 64)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
            except (BrokenPipeError, ConnectionResetError):
                client_alive = False
            finally:
                with _stream_lock:
                    if _active_ffmpeg is proc:
                        _stop_active_ffmpeg()
                    elif proc.poll() is None:
                        proc.kill()

            if client_alive:
                sys.stderr.write(
                    "FFmpeg exited while client still connected; restarting remux…\n"
                )


def main() -> None:
    parser = argparse.ArgumentParser(description="DroneLink localhost video viewer")
    parser.add_argument("--host", default=DEFAULT_HOST, help="bind address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="HTTP port")
    parser.add_argument(
        "--sdp",
        type=Path,
        default=DEFAULT_SDP,
        help="path to DroneLink.sdp (UDP RTP receiver description)",
    )
    args = parser.parse_args()

    sdp = args.sdp.resolve()
    if not sdp.is_file():
        raise SystemExit(f"SDP file not found: {sdp}")

    ffmpeg_bin = find_ffmpeg()
    ViewerHandler.sdp_path = sdp
    ViewerHandler.ffmpeg_bin = ffmpeg_bin

    server = ThreadingHTTPServer((args.host, args.port), ViewerHandler)
    url = f"http://{args.host}:{args.port}/"
    print(f"DroneLink viewer: {url}")
    print(f"SDP: {sdp}")
    print(f"FFmpeg: {ffmpeg_bin}")
    print("Waiting for RTP on the address/port in the SDP (default 192.168.50.2:5600).")
    print("Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        with _stream_lock:
            _stop_active_ffmpeg()
        server.server_close()


if __name__ == "__main__":
    main()
