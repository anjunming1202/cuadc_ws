#!/usr/bin/env python3
"""cuadc_onboard 可控相机服务：独占相机 → raw tee → 编码图传 + raw 给 CV。

- libcamerasrc 出 NV12 后立刻 tee：
  - 一路 x264 → 本地 H.264/RTP 5602(兼容读帧) + 5603(video-forward→地面)
    + 5605(onboard_record 全质量录制;无人读则内核丢弃)
  - 一路 shmsink raw NV12（任务感知直吃，无需再解码）
- 自己不发地面——干净图传 video-forward(5603→5600),CV 预览 cv-preview-ctl(按钮启停)。
- shm 每帧的拍照时刻(libcamera SensorTimestamp)经旁路 udp:127.0.0.1:5615 发出——shm 不带时间戳,
  感知按到达时刻配对取回(见 attach_capture_stamps)。
- 控制口(默认 udp:127.0.0.1:5610)收 JSON,PLAYING 态直接 g_object_set 到
  libcamerasrc,例如 {"ae":false,"exposure_us":1000,"gain":8.0,"af":"manual","lens":1.5}。
"""
from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import gi

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402

_AF_MODE = {"manual": 0, "auto": 1, "continuous": 2}
DEFAULT_RAW_SHM = "/tmp/cuadc-cam-raw.shm"
DEFAULT_STAMP_PORT = 5615
STAMP_FORMAT = "<qq"  # (发送时刻, 拍照时刻)，均为 CLOCK_MONOTONIC 纳秒


def _shm_bytes(w: int, h: int, n_buffers: int = 8) -> int:
    # NV12 ≈ 1.5 bytes/pixel; keep several slots for producer/consumer slack.
    return max(8_000_000, int(w * h * 3 // 2 * n_buffers))


def _cleanup_shm(path: str) -> None:
    """Remove stale shmsink sockets so a restart does not bind path.0 instead."""
    p = Path(path)
    for stale in (p, Path(str(p) + ".0"), Path(str(p) + ".1")):
        try:
            if stale.exists() or stale.is_symlink():
                stale.unlink()
                print(f"[shm] removed stale {stale}", flush=True)
        except OSError as exc:
            print(f"[shm] could not remove {stale}: {exc}", file=sys.stderr, flush=True)


def build_pipeline(
    w: int, h: int, fps: int, bitrate: int, raw_shm: str, cv_fps: int
) -> Gst.Pipeline:
    shm_size = _shm_bytes(w, h)
    desc = (
        f"libcamerasrc name=cam ! "
        f"video/x-raw,format=NV12,width={w},height={h},framerate={fps}/1 ! "
        f"tee name=raw "
        # Encode once for clean downlink (+ legacy 5602 H264 readers).
        f"raw. ! queue max-size-buffers=3 leaky=downstream ! "
        f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate} "
        f"key-int-max={fps} byte-stream=true bframes=0 rc-lookahead=0 "
        f"sync-lookahead=0 sliced-threads=true ! "
        f"h264parse config-interval=-1 ! rtph264pay config-interval=1 pt=96 ! "
        f"tee name=h "
        f"h. ! queue ! udpsink host=127.0.0.1 port=5602 sync=false async=false "
        f"h. ! queue ! udpsink host=127.0.0.1 port=5603 sync=false async=false "
        # 5605: same clean encode, tapped by onboard_record (full-quality). No
        # reader -> kernel drops it, so this branch is free when not recording.
        f"h. ! queue ! udpsink host=127.0.0.1 port=5605 sync=false async=false "
        # Raw NV12 for local CV — drop to cv_fps *before* shm (matches detect rate).
        f"raw. ! queue max-size-buffers=2 leaky=downstream ! "
        f"videorate drop-only=true ! video/x-raw,framerate={cv_fps}/1 ! "
        f"shmsink name=rawshm socket-path={raw_shm} shm-size={shm_size} "
        f"wait-for-connection=false sync=false async=false"
    )
    return Gst.parse_launch(desc)


def attach_capture_stamps(pipeline: Gst.Pipeline, port: int, sink_name: str = "rawshm",
                          host: str = "127.0.0.1") -> None:
    """每个写进 shm 的帧，发一条 (发送时刻, 拍照时刻) 到本机 UDP。libcamerasrc 的 PTS 由
    SensorTimestamp 换算而来，PTS + base_time = 拍照时刻（GstSystemClock = CLOCK_MONOTONIC）。
    shm 只传像素，读端用自己的到达时刻配对最近一条（帧间隔 100ms，两边只差 shm 一跳）。"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pad = pipeline.get_by_name(sink_name).get_static_pad("sink")

    def on_buffer(_pad, info):
        buf = info.get_buffer()
        if buf is not None and buf.pts != Gst.CLOCK_TIME_NONE:
            try:
                sock.sendto(struct.pack(STAMP_FORMAT, time.monotonic_ns(),
                                        pipeline.get_base_time() + buf.pts), (host, port))
            except OSError:
                pass
        return Gst.PadProbeReturn.OK

    pad.add_probe(Gst.PadProbeType.BUFFER, on_buffer)


def apply_controls(cam: Gst.Element, d: dict) -> None:
    """把一条 JSON 命令应用到 libcamerasrc。自动处理'手动曝光/对焦要先切 manual'。"""
    def setp(name, val):
        cam.set_property(name, val)

    # 曝光/增益:要生效必须先关 AE(把两个 mode 切 manual)
    if "exposure_us" in d or "gain" in d:
        if d.get("ae") is not False:
            setp("ae-enable", False)
    if "ae" in d:
        setp("ae-enable", bool(d["ae"]))
    if "exposure_us" in d:
        setp("exposure-time", int(d["exposure_us"]))
    if "gain" in d:
        setp("analogue-gain", float(d["gain"]))
    # 对焦:命令式手动;设 lens 前确保 af-mode=manual(不 hunting)
    if "lens" in d and "af" not in d:
        setp("af-mode", _AF_MODE["manual"])
    if "af" in d:
        setp("af-mode", _AF_MODE.get(str(d["af"]).lower(), 0))
    if "lens" in d:
        setp("lens-position", float(d["lens"]))
    # 其它 ISP/白平衡(可选,同名属性直传)
    for k in ("awb-enable", "sharpness", "contrast", "saturation", "brightness", "gamma"):
        jk = k.replace("-", "_")
        if jk in d:
            setp(k, d[jk])
    print(f"[ctrl] applied {d}", flush=True)


def control_listener(cam: Gst.Element, host: str, port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    print(f"[ctrl] listening udp {host}:{port}", flush=True)
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            d = json.loads(data.decode("utf-8"))
            if isinstance(d, dict):
                # 到主循环线程改属性,避免跨线程竞争
                GLib.idle_add(apply_controls, cam, d)
        except Exception as exc:  # 单条命令坏不该拖垮服务
            print(f"[ctrl] bad command: {exc}", file=sys.stderr, flush=True)


def main() -> None:
    ap = argparse.ArgumentParser(description="cuadc_onboard controllable camera service")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--bitrate", type=int, default=6000)
    ap.add_argument("--raw-shm", default=DEFAULT_RAW_SHM)
    ap.add_argument(
        "--cv-fps",
        type=int,
        default=10,
        help="raw shm branch rate for CV (detect/preview); clean encode stays at --fps",
    )
    ap.add_argument("--control-host", default="127.0.0.1")
    ap.add_argument("--control-port", type=int, default=5610)
    ap.add_argument("--stamp-port", type=int, default=DEFAULT_STAMP_PORT,
                    help="shm 帧拍照时刻旁路 UDP 端口；0 = 不发")
    args = ap.parse_args()

    Gst.init(None)
    _cleanup_shm(args.raw_shm)
    try:
        pipeline = build_pipeline(
            args.width,
            args.height,
            args.fps,
            args.bitrate,
            args.raw_shm,
            args.cv_fps,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"failed to build pipeline: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
    cam = pipeline.get_by_name("cam")
    if args.stamp_port:
        attach_capture_stamps(pipeline, args.stamp_port)
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

    threading.Thread(target=control_listener,
                     args=(cam, args.control_host, args.control_port),
                     daemon=True).start()

    pipeline.set_state(Gst.State.PLAYING)
    print(
        f"camera-service: {args.width}x{args.height}@{args.fps} "
        f"raw-tee -> h264 udp:5602+5603+5605 + shm:{args.raw_shm}@{args.cv_fps}fps; "
        f"ctrl {args.control_port}; capture stamps udp:{args.stamp_port or 'off'}",
        flush=True,
    )
    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()
