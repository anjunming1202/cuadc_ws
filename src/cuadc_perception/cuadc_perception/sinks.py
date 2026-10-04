"""结果出口（sink）：一次检测的结果扇出到多个出口。这里放**非-ROS** 的 RTP
预览 sink（gi/GStreamer，不依赖 rclpy）——把标注画到帧上、H.264 编码推到本地
udp（onboard 转发到地面），供真机预览和无-ROS 预览模式复用。ROS 的
DetectionArray/debug-Image 出口是 ROS 原生的、留在节点里（用 self.create_publisher）。

见 docs/spec/adr/0008-perception-raw-frame-multi-sink.md。
"""
from __future__ import annotations

import cv2
import numpy as np

DEFAULT_PREVIEW_PORT = 5604

_GST = None


def _ensure_gst():
    """惰性 import + init GStreamer（真机 RTP 预览时才用）：sim 不用 gstreamer，且
    Gst.init() 必须在创建 rclpy Node 之后、否则段错误（见 ADR-0009）。"""
    global _GST
    if _GST is None:
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        Gst.init(None)
        _GST = Gst
    return _GST


def draw_overlay(frame_bgr: np.ndarray, result, frame_idx: int, ema_ms: float) -> None:
    """把检测框 + 计时信息画到 frame 上（就地）。接受=绿圈+尺寸/置信标签，
    拒绝=灰圈。result 是 perception_core.PerceptionResult。"""
    for cr in result.contour_results:
        obs, det = cr.observation, cr.detection
        u, v, r = int(obs.u), int(obs.v), max(1, int(obs.radius_px))
        if det is not None:
            cv2.circle(frame_bgr, (u, v), r, (0, 220, 0), 2)
            cv2.circle(frame_bgr, (u, v), 3, (0, 255, 255), -1)
            label = f"{det.apparent_size:.2f}m {det.confidence:.2f}"
            cv2.putText(frame_bgr, label, (u + 6, max(16, v - r - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        else:
            cv2.circle(frame_bgr, (u, v), r, (128, 128, 128), 1)
    perf = result.perf or {}
    total_ms = perf.get('total_ms', 0.0)
    n_acc = len(result.detections)
    n_obs = len(result.observations)
    lines = [
        "OpenCV processed",
        f"obs={n_obs} acc={n_acc}  frame={total_ms:.0f}ms  ema={ema_ms:.0f}ms",
        f"n={frame_idx}",
    ]
    y = 22
    for line in lines:
        cv2.putText(frame_bgr, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame_bgr, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        y += 20


class RtpPreviewSink:
    """appsrc(BGR) → x264 → rtph264pay → udpsink 127.0.0.1:<port>。push(frame) 每帧一
    次。不依赖 rclpy。"""

    def __init__(self, port: int, width: int, height: int, fps: int, bitrate: int = 1000) -> None:
        self._gst = _ensure_gst()
        self._port = port
        self._w, self._h, self._fps = width, height, fps
        self._bitrate_kbps = bitrate
        self._pipeline = None
        self._src = None
        self._pts = 0
        self._frame_duration = self._gst.SECOND // max(1, fps)

    def start(self) -> None:
        desc = (
            f"appsrc name=src is-live=true format=time do-timestamp=true "
            f'caps="video/x-raw,format=BGR,width={self._w},height={self._h},'
            f'framerate={self._fps}/1" ! '
            f"queue max-size-buffers=1 leaky=downstream ! videoconvert ! "
            f"x264enc tune=zerolatency speed-preset=ultrafast bitrate={self._bitrate_kbps} "
            f"key-int-max={max(2, self._fps)} byte-stream=true bframes=0 "
            f"rc-lookahead=0 sync-lookahead=0 threads=2 ! "
            f"h264parse config-interval=1 ! rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host=127.0.0.1 port={self._port} sync=false async=false"
        )
        self._pipeline = self._gst.parse_launch(desc)
        self._src = self._pipeline.get_by_name("src")
        self._pipeline.set_state(self._gst.State.PLAYING)

    def push(self, frame_bgr: np.ndarray) -> None:
        if self._src is None:
            return
        data = frame_bgr.tobytes()
        buf = self._gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        buf.pts = self._pts
        buf.dts = self._pts
        buf.duration = self._frame_duration
        self._pts += self._frame_duration
        self._src.emit("push-buffer", buf)

    def stop(self) -> None:
        if self._pipeline is not None:
            self._pipeline.set_state(self._gst.State.NULL)
