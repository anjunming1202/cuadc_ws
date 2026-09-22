#!/usr/bin/env python3
"""CV preview producer: read camera raw NV12 from shm (no H264 decode),
burn-in label, re-encode H.264/RTP to :5604 for cv-forward → ground.

Does not open the camera. Camera-service tees raw to shm after capture
(already paced to detect rate, default 10 fps).
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import gi

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402

DEFAULT_RAW_SHM = "/tmp/cuadc-cam-raw.shm"


def build_pipeline(
    raw_shm: str,
    src_w: int,
    src_h: int,
    src_fps: int,
    dst_port: int,
    bitrate: int,
    label: str,
    width: int,
    height: int,
) -> Gst.Pipeline:
    # Camera shm branch is already @ src_fps (10). Scale then convert once.
    desc = (
        f"shmsrc socket-path={raw_shm} is-live=true do-timestamp=true ! "
        f"video/x-raw,format=NV12,width={src_w},height={src_h},"
        f"framerate={src_fps}/1 ! "
        f"queue max-size-buffers=1 leaky=downstream ! "
        f"videoscale method=nearest-neighbour ! "
        f"video/x-raw,width={width},height={height} ! "
        f"videoconvert ! "
        f'textoverlay text="{label}" valignment=top halignment=left '
        f'font-desc="Sans Bold 22" shaded-background=true '
        f"color=0xFFFFFFFF ! "
        f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate} "
        f"key-int-max={max(2, src_fps)} byte-stream=true bframes=0 "
        f"rc-lookahead=0 sync-lookahead=0 threads=2 ! "
        f"h264parse config-interval=1 ! rtph264pay config-interval=1 pt=96 ! "
        f"udpsink host=127.0.0.1 port={dst_port} sync=false async=false"
    )
    return Gst.parse_launch(desc)


def _wait_for_shm(path: str, timeout_s: float = 20.0) -> bool:
    """Wait until camera creates the shmsink socket path (do not connect —
    a raw connect confuses shmsink and kills the camera pipeline)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if Path(path).exists():
            time.sleep(0.5)  # let shmsink finish bind
            return True
        time.sleep(0.25)
    return False


def main() -> None:
    ap = argparse.ArgumentParser(description="CV preview: raw shm → labeled → 5604")
    ap.add_argument("--raw-shm", default=DEFAULT_RAW_SHM)
    ap.add_argument("--src-width", type=int, default=1920)
    ap.add_argument("--src-height", type=int, default=1080)
    ap.add_argument("--src-fps", type=int, default=10)  # match camera --cv-fps
    ap.add_argument("--dst-port", type=int, default=5604)
    ap.add_argument("--bitrate", type=int, default=1000)
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=540)
    ap.add_argument(
        "--label",
        default="OpenCV processed",
        help="burn-in text on the CV preview stream",
    )
    args = ap.parse_args()

    if not _wait_for_shm(args.raw_shm):
        print(f"shm not ready: {args.raw_shm}", file=sys.stderr)
        raise SystemExit(1)

    Gst.init(None)
    try:
        pipeline = build_pipeline(
            args.raw_shm,
            args.src_width,
            args.src_height,
            args.src_fps,
            args.dst_port,
            args.bitrate,
            args.label,
            args.width,
            args.height,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"failed to build pipeline: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc

    loop = GLib.MainLoop()

    def on_bus(_bus, msg):
        if msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print(f"[gst] ERROR {err}: {dbg}", file=sys.stderr, flush=True)
            loop.quit()
        elif msg.type == Gst.MessageType.WARNING:
            err, dbg = msg.parse_warning()
            print(f"[gst] WARN {err}: {dbg}", file=sys.stderr, flush=True)
        elif msg.type == Gst.MessageType.EOS:
            print("[gst] EOS", file=sys.stderr, flush=True)
            loop.quit()
        return True

    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_bus)

    pipeline.set_state(Gst.State.PLAYING)
    print(
        f"cv-preview: shm:{args.raw_shm} ({args.src_width}x{args.src_height}@"
        f"{args.src_fps}) -> {args.width}x{args.height}@{args.src_fps} label "
        f"-> udp:127.0.0.1:{args.dst_port} bitrate={args.bitrate}",
        flush=True,
    )
    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()
