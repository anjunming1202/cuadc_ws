"""sim-only 相机成像扰动节点（gazebo_bringup 里，相机桥之后）：按真机实测给渲染图加镜头/ISP
柔化（高斯 blur_sigma_px）和暗角（×(1−k·r²)），再延迟 delay_ms 转发，**原样保留拍照时间戳**——
复现真机"旧帧+真拍照戳"。参数全 0 = 原样直通。量值出处与红线见 docs/spec/sim-stress-testbed.md §3.2。"""

from array import array
from collections import deque

import cv2
import numpy as np

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageDelayNode(Node):

    def __init__(self):
        super().__init__('image_delay')
        self.declare_parameter('delay_ms', 0.0,
                               ParameterDescriptor(dynamic_typing=True))  # 150 与 150.0 都收
        self._delay = Duration(seconds=max(0.0, float(self.get_parameter('delay_ms').value) / 1000.0))
        self._blur = float(self.declare_parameter('blur_sigma_px', 0.0,
                                                  ParameterDescriptor(dynamic_typing=True)).value)
        self._vig_k = float(self.declare_parameter('vignetting_k', 0.0,
                                                   ParameterDescriptor(dynamic_typing=True)).value)
        self._vig_gain = None  # 按分辨率懒建的暗角增益图
        if self._blur > 0.0 or self._vig_k > 0.0:
            self.get_logger().info(f'[CAMERA] blur sigma {self._blur:.2f}px, vignetting k {self._vig_k:.3f}')
        self._pub = self.create_publisher(Image, 'image_out', 10)
        self.create_subscription(Image, 'image_in', self._on_msg, 10)
        if self._delay.nanoseconds > 0:
            self._buf = deque()
            self.create_timer(0.01, self._flush)  # 节点时钟(可 use_sim_time),RTF<1 也按仿真时间延迟
            self.get_logger().info(
                f'[LATENCY] images delayed {self._delay.nanoseconds / 1e6:.0f}ms (stamp preserved)')

    def _degrade(self, msg):
        if (self._blur <= 0.0 and self._vig_k <= 0.0) or msg.encoding not in ('rgb8', 'bgr8', 'mono8'):
            return msg
        ch = 1 if msg.encoding == 'mono8' else 3
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
        img = img.reshape(msg.height, msg.width, ch)
        if self._blur > 0.0:
            img = cv2.GaussianBlur(img, (0, 0), self._blur).reshape(msg.height, msg.width, ch)
        if self._vig_k > 0.0:
            if self._vig_gain is None or self._vig_gain.shape[:2] != (msg.height, msg.width):
                yy, xx = np.mgrid[0:msg.height, 0:msg.width].astype(np.float32)
                r2 = ((xx - msg.width / 2) ** 2 + (yy - msg.height / 2) ** 2) / (msg.width / 2) ** 2
                gain = 1.0 - self._vig_k * r2  # r 按半宽归一（实测口径）
                self._vig_gain = np.round(gain * 256).astype(np.uint16)[:, :, None]
            img = ((img.astype(np.uint16) * self._vig_gain + 128) >> 8).astype(np.uint8)
        msg.data = array('B', np.ascontiguousarray(img).tobytes())  # 赋 bytes 会逐元素校验，慢两个数量级
        msg.step = msg.width * ch
        return msg

    def _on_msg(self, msg):
        msg = self._degrade(msg)
        if self._delay.nanoseconds > 0:
            self._buf.append((self.get_clock().now() + self._delay, msg))
        else:
            self._pub.publish(msg)

    def _flush(self):
        now = self.get_clock().now()
        while self._buf and self._buf[0][0] <= now:
            self._pub.publish(self._buf.popleft()[1])


def main(args=None):
    rclpy.init(args=args)
    node = ImageDelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
