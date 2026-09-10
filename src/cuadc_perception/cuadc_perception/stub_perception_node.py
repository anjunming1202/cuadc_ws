"""桩感知节点：不订阅相机，直接从场地已知的圆筒坐标发布检测结果，供
决策层在真实感知算法完成前先打通全流程闭环。发布契约（话题名、消息
字段含义、无状态/逐帧独立）见 docs/spec/05-perception-decision-interface.md
——决策层不需要区分对面是桩还是真感知，两者发布同一份契约。"""

import math
import random
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Header

from cuadc_perception_msgs.msg import Detection, DetectionArray

# 圆筒地面真值，需要跟 cuadc_sim/field/config.py 的 DROP_CYLINDERS /
# DROP_CYLINDER_POSITIONS / RECON_CYLINDER_DIAMETER /
# RECON_CYLINDER_POSITIONS 保持一致——那份文件是仿真场地生成的唯一权威
# 源。这里没有直接 import 它，是因为 field/ 目前不是一个能被其它 ROS2
# 包 import 的 Python 包（field/gen_world.py 等脚本用的是脚本内的
# `import config`，不是包导入），改动 cuadc_sim 的包结构不在这个分支
# 范围内，先接受这一份手工同步成本。

# (label, diameter_m)
_DROP_CYLINDERS = [('1', 0.15), ('2', 0.20), ('3', 0.25)]
_DROP_CYLINDER_POSITIONS = {
    '1': (31.0, -1.5),
    '2': (32.5, 1.5),
    '3': (34.0, 0.0),
}

_RECON_CYLINDER_DIAMETER = 0.20
_RECON_CYLINDER_POSITIONS = {
    'a': (55.8, -2.5),
    'b': (56.6, 1.8),
    'c': (57.5, -0.3),
    'd': (58.3, 2.5),
    'e': (59.2, -1.8),
}

_DETECTIONS_TOPIC = '/perception/detections'
_LOCAL_POSITION_TOPIC = '/cuadc_interface/telemetry/local_position'


def _world_xy_to_ned(world_x, world_y):
    """场地坐标（cuadc_sim/field/config.py 的 x=飞行前进方向、y=侧向）
    转成接口层 telemetry/local_position 用的 NED (north, east)——两者
    不是恒等映射，X/Y 是对调的。换算依据：仿真用的机型
    `~/ardupilot_gazebo/models/iris_with_gimbal/model.sdf` 里
    `gazeboXYZToNED`（roll=180°, yaw=90°）这个 ArduPilotPlugin 参数，
    换算出来是 NED north = Gazebo 世界系 Y、NED east = Gazebo 世界系 X。
    2026-07-19 实机测试（SITL+Gazebo）发现飞机往侧边飞，就是这里之前
    直接把场地 (x, y) 当 NED (x, y) 用、没做这个换算导致的。
    """
    return (world_y, world_x)


class StubPerceptionNode(Node):
    """从场地已知坐标发布检测结果，不订阅相机，逐帧独立、无状态——跟
    真实感知节点的契约完全一致（05-perception-decision-interface.md），
    决策层不需要区分对面是桩还是真感知。

    可视范围：只报告距离当前水平位置 `visibility_radius_m` 以内的圆筒
    ——真实相机不可能一直看到全场，桩感知如果不模拟这一点，
    `SURVEY_RECON_ZONE` 的扫描/覆盖逻辑就测不出来（反正一进侦察区第一帧
    就能看全 5 个筒）。在收到第一条 local_position 之前，不发布任何
    检测（还不知道当前在哪，没法判断可视范围）。
    """

    def __init__(self):
        super().__init__('cuadc_perception_stub')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('position_noise_stddev', 0.0)
        self.declare_parameter('visibility_radius_m', 2.5)

        self._publisher = self.create_publisher(DetectionArray, _DETECTIONS_TOPIC, 10)
        self._local_position_sub = self.create_subscription(
            PoseStamped, _LOCAL_POSITION_TOPIC, self._on_local_position, 10)
        self._current_xy = None

        rate_hz = self.get_parameter('publish_rate_hz').value
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

    def _on_local_position(self, msg):
        self._current_xy = (msg.pose.position.x, msg.pose.position.y)

    def _on_timer(self):
        if self._current_xy is None:
            return

        started = time.monotonic()
        cur_north, cur_east = self._current_xy
        radius = self.get_parameter('visibility_radius_m').value
        noise_stddev = self.get_parameter('position_noise_stddev').value

        detections = []
        for label, diameter in _DROP_CYLINDERS:
            north, east = _world_xy_to_ned(*_DROP_CYLINDER_POSITIONS[label])
            if math.hypot(north - cur_north, east - cur_east) <= radius:
                detections.append(self._make_detection(north, east, diameter / 2.0, noise_stddev))
        for world_x, world_y in _RECON_CYLINDER_POSITIONS.values():
            north, east = _world_xy_to_ned(world_x, world_y)
            if math.hypot(north - cur_north, east - cur_east) <= radius:
                detections.append(self._make_detection(north, east, _RECON_CYLINDER_DIAMETER / 2.0, noise_stddev))

        msg = DetectionArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.detections = detections
        self._publisher.publish(msg)

        # 桩节点开销可忽略，先把日志形状搭好——换真实 CV 后这里量出来的
        # 才是真正有意义的耗时。
        elapsed_ms = (time.monotonic() - started) * 1000.0
        self.get_logger().info(
            f'[PERF] frame built in {elapsed_ms:.2f}ms, n={len(detections)}',
            throttle_duration_sec=1.0)

    @staticmethod
    def _make_detection(north, east, radius, noise_stddev):
        detection = Detection()
        detection.target_type = 'cylinder'
        detection.position.x = north + (random.gauss(0.0, noise_stddev) if noise_stddev > 0.0 else 0.0)
        detection.position.y = east + (random.gauss(0.0, noise_stddev) if noise_stddev > 0.0 else 0.0)
        detection.position.z = 0.0
        detection.apparent_size = radius
        detection.confidence = 1.0
        return detection


def main(args=None):
    rclpy.init(args=args)
    node = StubPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
