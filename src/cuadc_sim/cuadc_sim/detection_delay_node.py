"""sim-only：把感知的 DetectionArray 延迟 delay_ms 再转发，建模真实检测流水线
时延（相机→CV→决策）。**只有压测台的 mission_sim_delayed.launch.py 会拉起它**
——生产链（mission_bringup）里根本没有这个节点，真机/正式仿真的感知→决策是零
延迟直连，扰动注入完全不进任何生产节点/生产 launch。这是把时延注入从生产节点
移出来的关键（见 docs/spec/sim-stress-testbed.md §3.2）。

订 `detections_in`、延迟后发 `detections_out`（两个话题名由 launch 里的 remap
绑定到真实的 /perception/detections 链路上）。delay_ms=0 时直通、不建缓冲/定时器。
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
