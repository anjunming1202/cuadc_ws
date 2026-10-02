"""sim-only 扰动节点：把相机 Image 延迟 delay_ms 再转发，**原样保留拍照时间戳**——
复现真机"旧帧+真拍照戳"（相机→CV 链路时延）。生产链没有它，只由量测/压测工具
opt-in 插入；角色与红线见 docs/spec/sim-stress-testbed.md §1.1/§3.2/§3.3。"""

from collections import deque

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
        self._pub = self.create_publisher(Image, 'image_out', 10)
        self.create_subscription(Image, 'image_in', self._on_msg, 10)
        if self._delay.nanoseconds > 0:
            self._buf = deque()
            self.create_timer(0.01, self._flush)  # 节点时钟(可 use_sim_time),RTF<1 也按仿真时间延迟
            self.get_logger().info(
                f'[LATENCY] images delayed {self._delay.nanoseconds / 1e6:.0f}ms (stamp preserved)')

    def _on_msg(self, msg):
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
