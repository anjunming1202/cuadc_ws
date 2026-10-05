"""桩感知节点：不订阅相机，直接从场地已知的圆筒坐标发布检测结果，供
决策层在真实感知算法完成前先打通全流程闭环。发布契约（话题名、消息
字段含义、无状态/逐帧独立）见 docs/spec/05-perception-decision-interface.md
——决策层不需要区分对面是桩还是真感知，两者发布同一份契约。"""

import math
import random
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

import cuadc_physical
from cuadc_perception import geometry
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
_ATTITUDE_TOPIC = '/cuadc_interface/telemetry/attitude'

# 桩也要发 bearing（末端 IBVS 用，见 05 契约）。前向投影用：内参对方位方向
# 无影响（方位=射线在相机系的比值），只需默认 nadir 挂载；圆筒顶面在地面（起飞坐标系
# down = 飞控离地高）之上一个筒高，跟真实感知反投影到筒顶平面一致。
_CAM_MOUNT = geometry.CameraParams(fx=1.0, fy=1.0, cx=0.0, cy=0.0)
_CYLINDER_TOP_DOWN_M = (cuadc_physical.load_airframe().fc_rest_clearance_m
                        - cuadc_physical.load_targets().drop_cylinder_height_m)


def _world_xy_to_takeoff(world_x, world_y):
    """仿真世界（Gazebo / field/config.py，x=前进方向、y=左）坐标 -> 起飞坐标系 (前, 右)。仿真飞机出生在
    起降点 (0, 0)、机头朝世界 +x，所以 前 = 世界 x、右 = −世界 y（见 docs/spec/coordinate-frames.md）。
    忽略 EKF 航向估计与真朝向的 1–2° 偏差（桩只供决策打通闭环）。"""
    return (world_x, -world_y)


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
        self.create_subscription(Imu, _ATTITUDE_TOPIC, self._on_attitude, 10)
        self._current_xy = None
        self._pos = None   # 起飞坐标系 (前, 右, 下)，前向投影 bearing 用
        self._orient = None    # (x, y, z, w) 机体->世界，前向投影 bearing 用

        rate_hz = self.get_parameter('publish_rate_hz').value
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

    def _on_local_position(self, msg):
        self._current_xy = (msg.pose.position.x, msg.pose.position.y)
        self._pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _on_attitude(self, msg):
        q = msg.orientation
        self._orient = (q.x, q.y, q.z, q.w)

    def _on_timer(self):
        if self._current_xy is None:
            return

        started = time.monotonic()
        cur_x, cur_y = self._current_xy
        radius = self.get_parameter('visibility_radius_m').value
        noise_stddev = self.get_parameter('position_noise_stddev').value

        detections = []
        for label, diameter in _DROP_CYLINDERS:
            x, y = _world_xy_to_takeoff(*_DROP_CYLINDER_POSITIONS[label])
            if math.hypot(x - cur_x, y - cur_y) <= radius:
                detections.append(self._make_detection(x, y, diameter / 2.0, noise_stddev))
        for world_x, world_y in _RECON_CYLINDER_POSITIONS.values():
            x, y = _world_xy_to_takeoff(world_x, world_y)
            if math.hypot(x - cur_x, y - cur_y) <= radius:
                detections.append(self._make_detection(x, y, _RECON_CYLINDER_DIAMETER / 2.0, noise_stddev))

        msg = DetectionArray()
        msg.header = Header()
        # 桩感知没有真图像："拍照时刻"=本 tick 时刻，vehicle_pose=当前遥测位姿
        # （同刻自洽，满足 DetectionArray 契约，下游不区分桩/真）。
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._pos is not None:
            (msg.vehicle_pose.position.x, msg.vehicle_pose.position.y,
             msg.vehicle_pose.position.z) = self._pos
        if self._orient is not None:
            (msg.vehicle_pose.orientation.x, msg.vehicle_pose.orientation.y,
             msg.vehicle_pose.orientation.z, msg.vehicle_pose.orientation.w) = self._orient
        msg.detections = detections
        self._publisher.publish(msg)

        # 桩节点开销可忽略，先把日志形状搭好——换真实 CV 后这里量出来的
        # 才是真正有意义的耗时。debug 级别默认不打印，不会刷屏；真要看
        # 就在 launch 时加 --log-level cuadc_perception_stub:=debug。
        elapsed_ms = (time.monotonic() - started) * 1000.0
        self.get_logger().debug(
            f'[PERF] frame built in {elapsed_ms:.2f}ms, n={len(detections)}',
            throttle_duration_sec=1.0)

    def _make_detection(self, x, y, radius, noise_stddev):
        detection = Detection()
        detection.target_type = 'cylinder'
        detection.position.x = x + (random.gauss(0.0, noise_stddev) if noise_stddev > 0.0 else 0.0)
        detection.position.y = y + (random.gauss(0.0, noise_stddev) if noise_stddev > 0.0 else 0.0)
        detection.position.z = 0.0
        detection.bearing_x, detection.bearing_y = self._bearing_of(x, y)
        detection.apparent_size = radius
        detection.confidence = 1.0
        return detection

    def _bearing_of(self, x, y):
        """前向投影圆筒顶中心 -> 归一化方位 (u-cx)/fx, (v-cy)/fy，与感知
        normalize_bearing 一致（方向只取决于射线在相机系的比值，与内参无关，
        所以决策 _bearing_to_off 能精确反出真实相对偏移）。缺姿态时回 (0,0)。"""
        if self._pos is None or self._orient is None:
            return (0.0, 0.0)
        pose = geometry.VehiclePose(position=self._pos, orientation_xyzw=self._orient)
        cam_to_world = geometry.cam_to_world_matrix(_CAM_MOUNT, pose)
        rel = np.array([x - self._pos[0], y - self._pos[1],
                        _CYLINDER_TOP_DOWN_M - self._pos[2]])
        cam_vec = cam_to_world.T @ rel  # world -> cam
        if cam_vec[2] <= 1e-3:
            return (0.0, 0.0)  # 目标在相机后方/接近成像平面，方位无意义
        return (float(cam_vec[0] / cam_vec[2]), float(cam_vec[1] / cam_vec[2]))


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
