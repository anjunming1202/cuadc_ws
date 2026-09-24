"""取帧源：把"原始帧从哪来"和检测解耦。这里放**非-ROS** 的 shm 源
（gi/GStreamer，不依赖 rclpy）——真机从 camera_service 的原始 NV12 shm 直接取帧、
零解码，也供无-ROS 预览模式复用。仿真的 `/camera/image_raw` 订阅是 ROS 原生的、
留在节点里（就是一个 create_subscription，不值得单独包一层）。

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
    """惰性 import + init GStreamer：只有真机真正用到 shm 取帧时才加载。不在模块级
    import、也不无条件 Gst.init——`Gst.init()` 若在创建 rclpy Node 之前运行会与之
    冲突、段错误，而 sim 走 ROS 图源、根本不用 gstreamer，惰性化后 sim 完全不碰它。"""
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


class ShmFrameSource:
    """读 camera_service 的原始 NV12 shm，缩放/转 BGR 后推给 on_frame(frame_bgr)。
    push 型：内部 worker 线程按 shm 帧率拉，每帧回调一次。不依赖 rclpy。"""

    def __init__(self, raw_shm: str, src_w: int, src_h: int, src_fps: int,
                 out_w: int, out_h: int) -> None:
        self._raw_shm = raw_shm
        self._src_w, self._src_h, self._src_fps = src_w, src_h, src_fps
        self._out_w, self._out_h = out_w, out_h
        self._pipeline = None
        self._sink = None
        self._gst = None  # 在 start() 里惰性拿到
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._on_frame = None

    def _build(self) -> None:
        desc = (
            f"shmsrc socket-path={self._raw_shm} is-live=true do-timestamp=true ! "
            f"video/x-raw,format=NV12,width={self._src_w},height={self._src_h},"
            f"framerate={self._src_fps}/1 ! "
            f"queue max-size-buffers=1 leaky=downstream ! "
            f"videoscale method=nearest-neighbour ! videoconvert ! "
            f"video/x-raw,format=BGR,width={self._out_w},height={self._out_h} ! "
            f"appsink name=sink max-buffers=1 drop=true sync=false emit-signals=false"
        )
        self._pipeline = self._gst.parse_launch(desc)
        self._sink = self._pipeline.get_by_name("sink")

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
        """on_frame(frame_bgr) 每帧回调一次。shm 未就绪返回 False。"""
        if not _wait_for_shm(self._raw_shm):
            print(f"shm not ready: {self._raw_shm}", file=sys.stderr, flush=True)
            return False
        self._on_frame = on_frame
        self._gst = _ensure_gst()
        self._build()
        self._pipeline.set_state(self._gst.State.PLAYING)
        self._thread = threading.Thread(target=self._loop, name="shm-frame-source", daemon=True)
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
            self._pipeline.set_state(Gst.State.NULL)
