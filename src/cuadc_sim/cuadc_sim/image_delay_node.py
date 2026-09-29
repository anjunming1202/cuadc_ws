"""sim-only 扰动节点：把相机 Image 延迟 delay_ms 再转发，**原样保留 header.stamp
（拍照时刻）**——精确复现真机"拿到的是旧帧+真拍照戳"的形态（相机→CV 链路时延，
量值取 cv-field-test profiling 实测档）。生产链没有它，只有量测/压测工具插入
（遵守 sim-stress-testbed §1.1 红线：sim-only、opt-in、默认不存在）。

与 detection_delay_node 的关系见 sim-stress-testbed §3.3：那个延迟"结果到决策"，
这个延迟"图像进感知"——后者才会触发反投影/IBVS 的位姿-图像时刻不一致，用来验证
拍照时刻对齐(ADR-0010 B)是否把散点压住。
"""

import time
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageDelayNode(Node):

    def __init__(self):
        super().__init__('image_delay')
        self.declare_parameter('delay_ms', 0.0)
        self._delay_s = max(0.0, self.get_parameter('delay_ms').value / 1000.0)
        self._pub = self.create_publisher(Image, 'image_out', 10)
        self.create_subscription(Image, 'image_in', self._on_msg, 10)
        if self._delay_s > 0.0:
            self._buf = deque()
            self.create_timer(0.01, self._flush)  # 100Hz 冲刷,延迟粒度 10ms
            self.get_logger().info(f'[LATENCY] images delayed {self._delay_s * 1000:.0f}ms (stamp preserved)')

    def _on_msg(self, msg):
        if self._delay_s > 0.0:
            self._buf.append((time.monotonic() + self._delay_s, msg))
        else:
            self._pub.publish(msg)

    def _flush(self):
        now = time.monotonic()
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
