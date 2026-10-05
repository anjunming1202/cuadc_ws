"""CV 预览（无-ROS 模式）：从相机服务取帧（默认原始 shm；shmsrc 接不上时
`--from-rtp-port 5602` 回退本地 H.264 tap）→ 跑选定检测器画框 → H.264/RTP 推本地
`--port`（转发到地面）。**不订位姿、不反投影、不发 ROS**——供相机自检/现场调参，
不需要起 ROS 栈。

和 cv_perception_node 共用同一套 `frame_sources` / `perception_core` / `sinks` /
`detectors`，只是接线不同（pose=None、只挂 RTP sink）。设计见
docs/spec/adr/0008-perception-raw-frame-multi-sink.md。
    python3 -m cuadc_perception.cv_preview            # 默认 real 档、shm→5604
"""
from __future__ import annotations

import argparse
import signal
import sys
import threading

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

from cuadc_perception import detectors
from cuadc_perception.frame_sources import DEFAULT_RAW_SHM, RtpFrameSource, ShmFrameSource
from cuadc_perception.perception_core import PerceptionCore
from cuadc_perception.sinks import DEFAULT_PREVIEW_PORT, RtpPreviewSink, draw_overlay


def main() -> None:
    ap = argparse.ArgumentParser(description="CV 预览(无 ROS):shm→检测画框→RTP")
    ap.add_argument("--raw-shm", default=DEFAULT_RAW_SHM)
    ap.add_argument("--src-width", type=int, default=1920)
    ap.add_argument("--src-height", type=int, default=1080)
    ap.add_argument("--src-fps", type=int, default=10)
    ap.add_argument("--width", type=int, default=960, help="检测/预览分辨率")
    ap.add_argument("--height", type=int, default=540)
    ap.add_argument("--port", type=int, default=DEFAULT_PREVIEW_PORT)
    ap.add_argument("--bitrate", type=int, default=1000)
    ap.add_argument("--detector", default=detectors.ACTIVE_DEFAULT)
    ap.add_argument("--profile", default="real", choices=("real", "sim"))
    ap.add_argument("--from-rtp-port", type=int, default=0,
                    help="设为 5602 时回退本地 H.264 tap 取帧（shmsrc 接不上时）")
    args = ap.parse_args()

    Gst.init(None)
    detector = detectors.DETECTORS[args.detector]
    cfg = detector.load_config(args.profile)
    core = PerceptionCore(detector, cfg)  # 预览:pose=None,无需相机内参/半径门

    sink = RtpPreviewSink(args.port, args.width, args.height, args.src_fps, args.bitrate)
    sink.start()

    state = {"n": 0, "ema": 0.0}

    def on_frame(frame_bgr):
        result = core.process(frame_bgr, pose=None, want_debug=False)
        state["n"] += 1
        total = result.perf.get("total_ms", 0.0)
        state["ema"] = total if state["ema"] <= 0 else 0.8 * state["ema"] + 0.2 * total
        draw_overlay(frame_bgr, result, state["n"], state["ema"])  # frame 是本地拷贝,可就地画
        sink.push(frame_bgr)
        if state["n"] == 1 or state["n"] % 30 == 0:
            print(f"cv-preview n={state['n']} obs={len(result.observations)} "
                  f"total={total:.1f}ms ema={state['ema']:.1f}ms", flush=True)

    if args.from_rtp_port > 0:
        src = RtpFrameSource(args.from_rtp_port, args.src_fps, args.width, args.height)
        src_desc = f"rtp:{args.from_rtp_port}"
    else:
        src = ShmFrameSource(args.raw_shm, args.src_width, args.src_height, args.src_fps,
                             args.width, args.height)
        src_desc = f"shm:{args.raw_shm}"
    if not src.start(on_frame):
        print("取帧源启动失败(相机服务在跑吗?)", file=sys.stderr)
        raise SystemExit(1)

    print(f"cv-preview: {src_desc} -> {args.detector}({args.profile}) "
          f"{args.width}x{args.height}@{args.src_fps} -> udp:127.0.0.1:{args.port}", flush=True)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        stop.wait()
    finally:
        src.stop()
        sink.stop()
        print("\n[cv-preview] 收尾完成", flush=True)


if __name__ == "__main__":
    main()
