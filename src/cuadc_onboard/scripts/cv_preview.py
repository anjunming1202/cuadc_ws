#!/usr/bin/env python3
"""CV preview producer (step 1): read local camera RTP :5602, burn-in label,
re-encode H.264/RTP to :5604 for cuadc-onboard-cv-forward → ground.

Does not open the camera. Later step 2 replaces the label-only path with
detection overlay + paced drop-frame processing.

Defaults intentionally lighter than the camera tee (scale + rate-limit): a
full 1080p30 decode+re-encode on this Pi saturates CPU and the ground player
then reconnect-loops on stalls.
"""
from __future__ import annotations

import argparse
import sys

import gi

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402


def build_pipeline(
    src_port: int,
    dst_port: int,
    bitrate: int,
    label: str,
    width: int,
    height: int,
    max_fps: int,
) -> Gst.Pipeline:
    # openh264dec: what this Pi image ships; avdec_h264 may be missing.
    # Scale + videorate before overlay/encode to keep CPU off the cliff.
    desc = (
        f"udpsrc port={src_port} "
        f'caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! '
        f"rtpjitterbuffer latency=80 ! rtph264depay ! h264parse ! openh264dec ! "
        f"videoconvert ! videoscale ! "
        f"video/x-raw,width={width},height={height} ! "
        f"videorate drop-only=true ! video/x-raw,framerate={max_fps}/1 ! "
        f'textoverlay text="{label}" valignment=top halignment=left '
        f'font-desc="Sans Bold 22" shaded-background=true '
        f"color=0xFFFFFFFF ! "
        f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate} "
        f"key-int-max={max(5, max_fps)} byte-stream=true bframes=0 "
        f"rc-lookahead=0 sync-lookahead=0 threads=2 ! "
        f"h264parse config-interval=-1 ! rtph264pay config-interval=1 pt=96 ! "
        f"udpsink host=127.0.0.1 port={dst_port} sync=false async=false"
    )
    return Gst.parse_launch(desc)


def main() -> None:
    ap = argparse.ArgumentParser(description="CV preview: 5602 → labeled → 5604")
    ap.add_argument("--src-port", type=int, default=5602)
    ap.add_argument("--dst-port", type=int, default=5604)
    ap.add_argument("--bitrate", type=int, default=1500)
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=540)
    ap.add_argument("--max-fps", type=int, default=15)
    ap.add_argument(
        "--label",
        default="OpenCV processed",
        help="burn-in text on the CV preview stream",
    )
    args = ap.parse_args()

    Gst.init(None)
    try:
        pipeline = build_pipeline(
            args.src_port,
            args.dst_port,
            args.bitrate,
            args.label,
            args.width,
            args.height,
            args.max_fps,
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
        elif msg.type == Gst.MessageType.EOS:
            print("[gst] EOS", file=sys.stderr, flush=True)
            loop.quit()
        return True

    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_bus)

    pipeline.set_state(Gst.State.PLAYING)
    print(
        f"cv-preview: udp:{args.src_port} -> {args.width}x{args.height}@"
        f"{args.max_fps} label -> udp:127.0.0.1:{args.dst_port} "
        f"bitrate={args.bitrate}",
        flush=True,
    )
    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()
