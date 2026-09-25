"""sim-only 压测台节点：把 DetectionArray 延迟 delay_ms 再转发，建模检测流水线时延。
生产链里没有它——只有 mission_sim_delayed.launch.py 拉起。订 detections_in、延迟后发
detections_out（话题名由 launch remap 绑定）；delay_ms=0 直通、不建缓冲/定时器。
"""

import time
from collections import deque

import rclpy
from rclpy.node import Node

from cuadc_perception_msgs.msg import DetectionArray


class DetectionDelayNode(Node):

    def __init__(self):
        super().__init__('detection_delay')
        self.declare_parameter('delay_ms', 0.0)
        self._delay_s = max(0.0, self.get_parameter('delay_ms').value / 1000.0)
        self._pub = self.create_publisher(DetectionArray, 'detections_out', 10)
        self.create_subscription(DetectionArray, 'detections_in', self._on_msg, 10)
        if self._delay_s > 0.0:
            self._buf = deque()  # [(due_monotonic, DetectionArray), ...]
            self.create_timer(0.02, self._flush)  # 50Hz 冲刷到期的检测
            self.get_logger().info(f'[LATENCY] detections delayed {self._delay_s * 1000:.0f}ms')

    def _on_msg(self, msg):
        if self._delay_s > 0.0:
            self._buf.append((time.monotonic() + self._delay_s, msg))
        else:
            self._pub.publish(msg)  # 0 = 直通

    def _flush(self):
        now = time.monotonic()
        while self._buf and self._buf[0][0] <= now:
            self._pub.publish(self._buf.popleft()[1])


def main(args=None):
    rclpy.init(args=args)
    node = DetectionDelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
