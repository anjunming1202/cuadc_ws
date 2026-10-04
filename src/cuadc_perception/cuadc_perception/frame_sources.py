"""取帧源：把"原始帧从哪来"和检测解耦。这里放**非-ROS** 的源
（gi/GStreamer，不依赖 rclpy）——真机优先从 camera_service 的原始 NV12 shm 直接
取帧、零解码；shmsrc 偶发接不上（posix shm 名已 unlink）时回退本地 H.264 tap
（5602）解码取帧。仿真的 `/camera/image_raw` 订阅是 ROS 原生的、留在节点里
（就是一个 create_subscription，不值得单独包一层）。

见 docs/spec/adr/0008-perception-raw-frame-multi-sink.md。扁平放；出现第 3 个源
再提成 frame_sources/ 目录（对齐 detectors/ 风格）。
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import numpy as np

DEFAULT_RAW_SHM = "/tmp/cuadc-cam-raw.shm"

_GST = None


def _ensure_gst():
    """惰性 import + init GStreamer（真机 shm 取帧时才用）：sim 不用 gstreamer，且
    Gst.init() 必须在创建 rclpy Node 之后、否则段错误（见 ADR-0009）。"""
    global _GST
    if _GST is None:
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        Gst.init(None)
        _GST = Gst
    return _GST


def _wait_for_shm(path: str, timeout_s: float = 20.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if Path(path).exists():
            time.sleep(0.5)
            return True
        time.sleep(0.25)
    return False


class _AppsinkFrameSource:
    """appsink 取帧共性：worker 线程按源帧率 try-pull、转 BGR 后推给
    on_frame(frame_bgr)。push 型，每帧回调一次。不依赖 rclpy。
    子类只给管线描述（`_describe`）和就绪检查（`_ready`）。"""

    def __init__(self, out_w: int, out_h: int) -> None:
        self._out_w, self._out_h = out_w, out_h
        self._pipeline = None
        self._sink = None
        self._gst = None  # 在 start() 里惰性拿到
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._on_frame = None

    def _describe(self) -> str:
        raise NotImplementedError

    def _ready(self) -> bool:
        return True

    def _to_bgr(self, sample) -> np.ndarray | None:
        buf = sample.get_buffer()
        if buf is None:
            return None
        ok, info = buf.map(self._gst.MapFlags.READ)
        if not ok:
            return None
        try:
            expected = self._out_w * self._out_h * 3
            if info.size < expected:
                return None
            return np.frombuffer(info.data, dtype=np.uint8, count=expected).reshape(
                (self._out_h, self._out_w, 3)).copy()
        finally:
            buf.unmap(info)

    def start(self, on_frame) -> bool:
        """on_frame(frame_bgr) 每帧回调一次。源未就绪返回 False。"""
        if not self._ready():
            return False
        self._on_frame = on_frame
        self._gst = _ensure_gst()
        self._pipeline = self._gst.parse_launch(self._describe())
        self._sink = self._pipeline.get_by_name("sink")
        self._pipeline.set_state(self._gst.State.PLAYING)
        self._thread = threading.Thread(target=self._loop, name="frame-source", daemon=True)
        self._thread.start()
        return True

    def _loop(self) -> None:
        while not self._stop.is_set():
            sample = self._sink.emit("try-pull-sample", self._gst.SECOND // 5)
            if sample is None:
                continue
            frame = self._to_bgr(sample)
            if frame is not None and self._on_frame is not None:
                self._on_frame(frame)

    def stop(self) -> None:
        self._stop.set()
        if self._pipeline is not None:
            self._pipeline.set_state(self._gst.State.NULL)


class ShmFrameSource(_AppsinkFrameSource):
    """读 camera_service 的原始 NV12 shm，缩放/转 BGR。首选源：零解码。"""

    def __init__(self, raw_shm: str, src_w: int, src_h: int, src_fps: int,
                 out_w: int, out_h: int) -> None:
        super().__init__(out_w, out_h)
        self._raw_shm = raw_shm
        self._src_w, self._src_h, self._src_fps = src_w, src_h, src_fps

    def _ready(self) -> bool:
        if _wait_for_shm(self._raw_shm):
            return True
        print(f"shm not ready: {self._raw_shm}", file=sys.stderr, flush=True)
        return False

    def _describe(self) -> str:
        return (
            f"shmsrc socket-path={self._raw_shm} is-live=true do-timestamp=true ! "
            f"video/x-raw,format=NV12,width={self._src_w},height={self._src_h},"
            f"framerate={self._src_fps}/1 ! "
            f"queue max-size-buffers=1 leaky=downstream ! "
            f"videoscale method=nearest-neighbour ! videoconvert ! "
            f"video/x-raw,format=BGR,width={self._out_w},height={self._out_h} ! "
            f"appsink name=sink max-buffers=1 drop=true sync=false emit-signals=false"
        )


class RtpFrameSource(_AppsinkFrameSource):
    """读本地 H.264 RTP tap（相机服务 tee 的 5602），解码 + 降帧到 fps。
    shmsrc 接不上时的回退源（2026-10-03 现场验证的权宜取帧口）。"""

    def __init__(self, port: int, fps: int, out_w: int, out_h: int) -> None:
        super().__init__(out_w, out_h)
        self._port = port
        self._fps = fps

    def _describe(self) -> str:
        return (
            f"udpsrc port={self._port} "
            f'caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! '
            f"rtpjitterbuffer latency=80 ! rtph264depay ! h264parse ! openh264dec ! "
            f"queue max-size-buffers=1 leaky=downstream ! "
            f"videorate drop-only=true ! video/x-raw,framerate={self._fps}/1 ! "
            f"videoscale method=nearest-neighbour ! videoconvert ! "
            f"video/x-raw,format=BGR,width={self._out_w},height={self._out_h} ! "
            f"appsink name=sink max-buffers=1 drop=true sync=false emit-signals=false"
        )
