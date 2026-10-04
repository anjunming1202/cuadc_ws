#!/usr/bin/env python3
"""CV preview (step 2): raw shm → color_pad detect @10fps → boxes + timing
overlay → H.264/RTP :5604 for cv-forward → ground.

Does not open the camera. Reads camera-service NV12 shm (already paced to
--src-fps). Detection uses cuadc_perception color_pad (real profile).
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from pathlib import Path

import cv2
import gi
import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402

DEFAULT_RAW_SHM = "/tmp/cuadc-cam-raw.shm"
DEFAULT_PERCEPTION_ROOT = "/home/wly/cuadc_ws/src/cuadc_perception"


def _import_color_pad(perception_root: str):
    root = str(Path(perception_root).resolve())
    if root not in sys.path:
        sys.path.insert(0, root)
    from cuadc_perception.detectors import color_pad  # noqa: WPS433

    return color_pad


def _build_in_pipeline_shm(raw_shm: str, src_w: int, src_h: int, src_fps: int,
                           width: int, height: int) -> Gst.Pipeline:
    # Scale+convert in gst so appsink already delivers preview-sized BGR.
    desc = (
        f"shmsrc socket-path={raw_shm} is-live=true do-timestamp=true ! "
        f"video/x-raw,format=NV12,width={src_w},height={src_h},"
        f"framerate={src_fps}/1 ! "
        f"queue max-size-buffers=1 leaky=downstream ! "
        f"videoscale method=nearest-neighbour ! "
        f"videoconvert ! "
        f"video/x-raw,format=BGR,width={width},height={height} ! "
        f"appsink name=sink max-buffers=1 drop=true sync=false emit-signals=false"
    )
    return Gst.parse_launch(desc)


def _build_in_pipeline_rtp(src_port: int, src_fps: int, width: int, height: int) -> Gst.Pipeline:
    # Fallback when GStreamer shmsrc cannot attach (posix shm name already unlinked).
    # Reads the local H.264 tap 5602; decode + drop to preview fps.
    desc = (
        f"udpsrc port={src_port} "
        f'caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! '
        f"rtpjitterbuffer latency=80 ! rtph264depay ! h264parse ! openh264dec ! "
        f"queue max-size-buffers=1 leaky=downstream ! "
        f"videorate drop-only=true ! video/x-raw,framerate={src_fps}/1 ! "
        f"videoscale method=nearest-neighbour ! videoconvert ! "
        f"video/x-raw,format=BGR,width={width},height={height} ! "
        f"appsink name=sink max-buffers=1 drop=true sync=false emit-signals=false"
    )
    return Gst.parse_launch(desc)


def _build_out_pipeline(dst_port: int, width: int, height: int, fps: int,
                        bitrate: int) -> Gst.Pipeline:
    desc = (
        f"appsrc name=src is-live=true format=time do-timestamp=true "
        f'caps="video/x-raw,format=BGR,width={width},height={height},'
        f'framerate={fps}/1" ! '
        f"queue max-size-buffers=1 leaky=downstream ! "
        f"videoconvert ! "
        f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate} "
        f"key-int-max={max(2, fps)} byte-stream=true bframes=0 "
        f"rc-lookahead=0 sync-lookahead=0 threads=2 ! "
        f"h264parse config-interval=1 ! rtph264pay config-interval=1 pt=96 ! "
        f"udpsink host=127.0.0.1 port={dst_port} sync=false async=false"
    )
    return Gst.parse_launch(desc)


def _buffer_to_bgr(sample: Gst.Sample, width: int, height: int) -> np.ndarray | None:
    buf = sample.get_buffer()
    if buf is None:
        return None
    ok, info = buf.map(Gst.MapFlags.READ)
    if not ok:
        return None
    try:
        expected = width * height * 3
        if info.size < expected:
            return None
        return np.frombuffer(info.data, dtype=np.uint8, count=expected).reshape(
            (height, width, 3)
        ).copy()
    finally:
        buf.unmap(info)


def _bgr_to_buffer(frame: np.ndarray, pts_ns: int) -> Gst.Buffer:
    data = frame.tobytes()
    buf = Gst.Buffer.new_allocate(None, len(data), None)
    buf.fill(0, data)
    buf.pts = pts_ns
    buf.dts = pts_ns
    buf.duration = Gst.SECOND // 10  # overwritten by caller if needed
    return buf


def _draw_detections(frame: np.ndarray, observations) -> None:
    for obs in observations:
        u, v, r = int(obs.u), int(obs.v), max(1, int(obs.radius_px))
        cv2.circle(frame, (u, v), r, (0, 220, 0), 2)
        cv2.circle(frame, (u, v), 3, (0, 255, 255), -1)
        cv2.putText(
            frame,
            f"{obs.circularity:.2f}",
            (u + 6, max(16, v - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )


def _draw_timing(
    frame: np.ndarray,
    n_det: int,
    detect_ms: float,
    total_ms: float,
    ema_ms: float,
    frame_idx: int,
) -> None:
    lines = [
        "OpenCV processed",
        f"det={n_det}  detect={detect_ms:.0f}ms  "
        f"frame={total_ms:.0f}ms  ema={ema_ms:.0f}ms",
        f"n={frame_idx}",
    ]
    y = 22
    for line in lines:
        cv2.putText(
            frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA
        )
        cv2.putText(
            frame, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA
        )
        y += 20


def _wait_for_shm(path: str, timeout_s: float = 20.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if Path(path).exists():
            time.sleep(0.5)
            return True
        time.sleep(0.25)
    return False


def main() -> None:
    ap = argparse.ArgumentParser(description="CV preview: detect + boxes → 5604")
    ap.add_argument("--raw-shm", default=DEFAULT_RAW_SHM)
    ap.add_argument("--src-width", type=int, default=1920)
    ap.add_argument("--src-height", type=int, default=1080)
    ap.add_argument("--src-fps", type=int, default=10)
    ap.add_argument("--dst-port", type=int, default=5604)
    ap.add_argument("--bitrate", type=int, default=1000)
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=540)
    ap.add_argument("--profile", default="real", choices=("real", "sim"))
    ap.add_argument(
        "--perception-root",
        default=os.environ.get("CUADC_PERCEPTION_ROOT", DEFAULT_PERCEPTION_ROOT),
        help="directory that contains the cuadc_perception package",
    )
    ap.add_argument(
        "--from-rtp-port",
        type=int,
        default=0,
        help="if set (e.g. 5602), decode local H.264 instead of raw shm",
    )
    args = ap.parse_args()

    color_pad = _import_color_pad(args.perception_root)
    cfg = color_pad.load_config(args.profile)

    if args.from_rtp_port <= 0 and not _wait_for_shm(args.raw_shm):
        print(f"shm not ready: {args.raw_shm}", file=sys.stderr)
        raise SystemExit(1)

    Gst.init(None)
    try:
        if args.from_rtp_port > 0:
            pin = _build_in_pipeline_rtp(
                args.from_rtp_port, args.src_fps, args.width, args.height,
            )
        else:
            pin = _build_in_pipeline_shm(
                args.raw_shm, args.src_width, args.src_height, args.src_fps,
                args.width, args.height,
            )
        pout = _build_out_pipeline(
            args.dst_port, args.width, args.height, args.src_fps, args.bitrate,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"failed to build pipeline: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc

    sink = pin.get_by_name("sink")
    src = pout.get_by_name("src")
    loop = GLib.MainLoop()
    stop = threading.Event()
    frame_idx = 0
    ema_ms = 0.0
    frame_duration = Gst.SECOND // max(1, args.src_fps)

    def on_bus(_bus, msg, tag: str):
        if msg.type == Gst.MessageType.ERROR:
            err, dbg = msg.parse_error()
            print(f"[gst:{tag}] ERROR {err}: {dbg}", file=sys.stderr, flush=True)
            stop.set()
            loop.quit()
        elif msg.type == Gst.MessageType.WARNING:
            err, dbg = msg.parse_warning()
            print(f"[gst:{tag}] WARN {err}: {dbg}", file=sys.stderr, flush=True)
        elif msg.type == Gst.MessageType.EOS:
            print(f"[gst:{tag}] EOS", file=sys.stderr, flush=True)
            stop.set()
            loop.quit()
        return True

    for pipe, tag in ((pin, "in"), (pout, "out")):
        bus = pipe.get_bus()
        bus.add_signal_watch()
        bus.connect("message", on_bus, tag)

    def worker() -> None:
        nonlocal frame_idx, ema_ms
        pts = 0
        while not stop.is_set():
            sample = sink.emit("try-pull-sample", Gst.SECOND // 5)
            if sample is None:
                continue
            frame = _buffer_to_bgr(sample, args.width, args.height)
            if frame is None:
                continue

            t0 = time.monotonic()
            perf: dict = {}
            try:
                obs = color_pad.detect_cylinders_pixels(frame, cfg, perf=perf)
            except Exception as exc:  # noqa: BLE001
                print(f"[detect] {exc}", file=sys.stderr, flush=True)
                obs = []
            detect_ms = (time.monotonic() - t0) * 1000.0
            _draw_detections(frame, obs)
            total_ms = (time.monotonic() - t0) * 1000.0
            ema_ms = total_ms if ema_ms <= 0 else (0.8 * ema_ms + 0.2 * total_ms)
            frame_idx += 1
            _draw_timing(frame, len(obs), detect_ms, total_ms, ema_ms, frame_idx)

            buf = _bgr_to_buffer(frame, pts)
            buf.duration = frame_duration
            pts += frame_duration
            ret = src.emit("push-buffer", buf)
            if ret != Gst.FlowReturn.OK and frame_idx % 30 == 0:
                print(f"[appsrc] push={ret}", file=sys.stderr, flush=True)

            if frame_idx == 1 or frame_idx % 30 == 0:
                print(
                    f"cv-preview frame={frame_idx} det={len(obs)} "
                    f"detect={detect_ms:.1f}ms total={total_ms:.1f}ms "
                    f"ema={ema_ms:.1f}ms",
                    flush=True,
                )

    pin.set_state(Gst.State.PLAYING)
    pout.set_state(Gst.State.PLAYING)
    print(
        f"cv-preview: src={'rtp:'+str(args.from_rtp_port) if args.from_rtp_port else 'shm:'+args.raw_shm} "
        f"-> color_pad({args.profile}) "
        f"{args.width}x{args.height}@{args.src_fps} -> udp:127.0.0.1:{args.dst_port} "
        f"bitrate={args.bitrate} perception={args.perception_root}",
        flush=True,
    )
    threading.Thread(target=worker, name="cv-worker", daemon=True).start()
    try:
        loop.run()
    finally:
        stop.set()
        pin.set_state(Gst.State.NULL)
        pout.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()
