"""真实感知节点：订阅相机图像 + 遥测（位置/姿态），跑 `detectors/` 里选定的
检测器，发布跟桩节点同一份契约（docs/spec/05-perception-decision-interface.md）
的 DetectionArray——决策层不需要区分对面是桩还是这个节点。

这里只做 ROS2 接线（订阅/参数/发布），算法本身在 `detectors/`（纯函数，不依赖
rclpy，可以脱离节点单测），跟 mission_logic.py/decision_node.py 的分层原则一致。
用哪个检测器 = `detector` 参数、sim/real = `profile` 参数（检测器库约定见
cuadc_perception/README.md）。

按图像帧率处理（收到一帧处理一帧），不像桩节点那样用固定频率定时器
——桩节点没有真实数据源触发，只能自己定频率；这里数据源就是相机本身
的帧率（sim 里是 10Hz，见 gazebo_bringup.launch.py 的相机 update_rate）。
"""

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header, String

from cuadc_perception import detectors, geometry
from cuadc_perception_msgs.msg import Detection, DetectionArray

_DETECTIONS_TOPIC = '/perception/detections'
_IMAGE_TOPIC = '/camera/image_raw'
_LOCAL_POSITION_TOPIC = '/cuadc_interface/telemetry/local_position'
_ATTITUDE_TOPIC = '/cuadc_interface/telemetry/attitude'
_MISSION_STATE_TOPIC = '/cuadc_decision/mission_state'

# cuadc_decision.mission_logic.State.SURVEY_RECON_ZONE 的字符串值——这里
# 只比对字符串，不 import mission_logic，感知层不需要知道决策层的状态机
# 结构，只需要这个契约字符串（跟 decision_node.py 发布 mission_state 时
# 用的是同一个纯字符串一致）。
_RECON_ZONE_STATE = 'SURVEY_RECON_ZONE'

# 每种调试图像各自一个话题 + 一个布尔开关，可以同时开好几路——真正的
# 安全上限是 max_concurrent_debug_views 参数（见 __init__），不是这里的
# 列表本身。按这个顺序做优先级：超过上限时靠前的先保留。
# cuadc_viz/web/index.html 的调试图像格子是手抄的第二份（前端不能 import
# 这个 Python 常量），加/删一种调试图像要两边一起改，忘改一边不会报错，
# 只会在网页上多一个点了没反应的格子，或者少一个格子。
_DEBUG_VIEWS = ('annotated', 'contours', 'mask')


class CvPerceptionNode(Node):

    def __init__(self):
        super().__init__('cuadc_perception_cv')

        # 相机内参默认值是 sim 实测值（~/ardupilot_gazebo/models/gimbal_small_3d/model.sdf：
        # horizontal_fov=2.0rad, 640x480），fx=fy=(640/2)/tan(1.0)，
        # cx/cy 是图像中心；真机换真实标定值即可，不用改代码。
        self.declare_parameter('fx', 205.5)
        self.declare_parameter('fy', 205.5)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        # 云台固定挂载角，跟 gazebo_bringup.launch.py 的
        # camera_fixed_roll/pitch/yaw_rad 同名同默认值，两边要保持一致。
        self.declare_parameter('camera_fixed_roll_rad', 0.0)
        self.declare_parameter('camera_fixed_pitch_rad', 1.5708)
        self.declare_parameter('camera_fixed_yaw_rad', 0.0)
        self.declare_parameter('cylinder_height_m', 0.3)
        self.declare_parameter('min_radius_m', 0.05)
        self.declare_parameter('max_radius_m', 0.18)
        # 用哪个检测器 + sim/real 档:检测器算法在 detectors/,各自可调参数
        # (HSV 阈值/形态学/几何门)在 detectors/<detector>.<profile>.yaml 跟着
        # 方法走(见 cuadc_perception/README.md 检测器库)。profile 由场景启动
        # 脚本选(sim / real),不用改代码。
        self.declare_parameter('detector', detectors.ACTIVE_DEFAULT)
        self.declare_parameter('profile', 'sim')
        # 每种调试图像各自一个布尔开关，默认全关。运行时用
        # `ros2 param set /cuadc_perception_cv debug_<view>_enabled true`切换
        # （cuadc_viz 网页的格子走的也是这个参数服务），不用重启节点。用
        # 布尔而不是字符串枚举，是吸取上次的教训——launch 生成的 params
        # YAML 是 YAML 1.1，字符串参数裸写 off/no/false 这类词会被解析成
        # 布尔，跟声明的字符串类型冲突，实测直接把节点启动崩了
        # （InvalidParameterTypeException，见 docs/lessons.md）；布尔参数
        # 天然是布尔类型，不存在这个问题。
        for view in _DEBUG_VIEWS:
            self.declare_parameter(f'debug_{view}_enabled', False)
        # 真正的硬件保护在这——不管前端同时勾了几个，机上最多只算/只发
        # 这么多路。仿真/联调可以调大，真机部署默认保守（1），不用改前端
        # 代码，前端多勾也不会真的让机上多算。超过上限的按 _DEBUG_VIEWS
        # 的顺序丢弃靠后的，不报错，只打限流日志。
        self.declare_parameter('max_concurrent_debug_views', 4)

        # 选定检测器 + 载入它的 sim/real 配置(一次,启动时)。配置从
        # detectors/<detector>.<profile>.yaml 读,跟着方法走。
        detector_name = self.get_parameter('detector').value
        profile = self.get_parameter('profile').value
        self._detector = detectors.DETECTORS[detector_name]
        self._detector_cfg = self._detector.load_config(profile)
        self.get_logger().info(f'[DETECTOR] {detector_name} (profile={profile})')

        self._bridge = CvBridge()
        self._latest_position_ned = None  # (north, east, down)
        self._latest_orientation_xyzw = None
        self._recon_zone_active = False

        self._publisher = self.create_publisher(DetectionArray, _DETECTIONS_TOPIC, 10)
        self._perf_publisher = self.create_publisher(DiagnosticArray, '~/perf', 10)
        self._debug_image_publishers = {
            view: self.create_publisher(Image, f'~/debug/{view}', 10) for view in _DEBUG_VIEWS
        }
        self._recon_image_publisher = self.create_publisher(Image, '~/recon_image', 10)
        self.create_subscription(Image, _IMAGE_TOPIC, self._on_image, 10)
        self.create_subscription(PoseStamped, _LOCAL_POSITION_TOPIC, self._on_local_position, 10)
        self.create_subscription(Imu, _ATTITUDE_TOPIC, self._on_attitude, 10)
        self.create_subscription(String, _MISSION_STATE_TOPIC, self._on_mission_state, 10)

    def _on_mission_state(self, msg):
        self._recon_zone_active = (msg.data == _RECON_ZONE_STATE)

    def _on_local_position(self, msg):
        self._latest_position_ned = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _on_attitude(self, msg):
        q = msg.orientation
        self._latest_orientation_xyzw = (q.x, q.y, q.z, q.w)

    def _on_image(self, msg):
        if self._latest_position_ned is None or self._latest_orientation_xyzw is None:
            return  # 还没收到遥测，不知道飞机在哪，没法反投影

        image_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        camera = geometry.CameraParams(
            fx=self.get_parameter('fx').value,
            fy=self.get_parameter('fy').value,
            cx=self.get_parameter('cx').value,
            cy=self.get_parameter('cy').value,
            mount_roll_rad=self.get_parameter('camera_fixed_roll_rad').value,
            mount_pitch_rad=self.get_parameter('camera_fixed_pitch_rad').value,
            mount_yaw_rad=self.get_parameter('camera_fixed_yaw_rad').value,
        )
        pose = geometry.VehiclePose(
            position_ned=self._latest_position_ned,
            orientation_xyzw=self._latest_orientation_xyzw,
        )

        cylinder_height_m = self.get_parameter('cylinder_height_m').value
        min_radius_m = self.get_parameter('min_radius_m').value
        max_radius_m = self.get_parameter('max_radius_m').value

        active_views = self._active_debug_views()
        perf = {}
        debug = {} if active_views else None
        observations = self._detector.detect_cylinders_pixels(
            image_bgr, self._detector_cfg, perf=perf, debug=debug)

        # 组装：检测器出图像观测 -> geometry 反投影出世界坐标 + 归一化方位
        # bearing -> 物理半径过滤 -> 填成 Detection。accept/reject 在这里决定
        # （要用到反投影出的斜距和相机内参），检测器本身不掺和。
        cam_to_world = geometry.cam_to_world_matrix(camera, pose)
        t_reproject = time.monotonic()
        detection_msgs = []
        contour_results = []
        for obs in observations:
            world_xy, range_m = geometry.reproject_to_cylinder_top(
                obs.u, obs.v, camera, pose, cam_to_world, cylinder_height_m)
            det_msg = None
            if world_xy is not None:
                radius_m = obs.radius_px * range_m / camera.fx
                if min_radius_m <= radius_m <= max_radius_m:
                    bearing_x, bearing_y = geometry.normalize_bearing(obs.u, obs.v, camera)
                    det_msg = Detection()
                    det_msg.target_type = 'cylinder'
                    det_msg.position.x = float(world_xy[0])
                    det_msg.position.y = float(world_xy[1])
                    det_msg.position.z = 0.0
                    det_msg.bearing_x = float(bearing_x)
                    det_msg.bearing_y = float(bearing_y)
                    det_msg.apparent_size = float(radius_m)
                    det_msg.confidence = float(min(obs.circularity, 1.0))
                    detection_msgs.append(det_msg)
            contour_results.append((obs, det_msg))
        reproject_ms = (time.monotonic() - t_reproject) * 1000.0
        if perf:
            perf['fit_reproject_ms'] = perf.pop('fit_ms', 0.0) + reproject_ms
            perf['total_ms'] = (perf['segment_ms'] + perf['morphology_ms']
                                + perf['contours_ms'] + perf['fit_reproject_ms'])
        if debug is not None:
            debug['contour_results'] = contour_results

        for view in active_views:
            self._publish_debug_image(view, image_bgr, debug, msg.header)
        if self._recon_zone_active:
            self._recon_image_publisher.publish(msg)

        out = DetectionArray()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.detections = detection_msgs
        self._publisher.publish(out)

        self._publish_perf(perf)

    def _active_debug_views(self):
        """按 _DEBUG_VIEWS 的固定优先级排，超过 max_concurrent_debug_views
        的靠后的直接丢——这是真机上唯一的硬性保护，不管前端请求了几路。"""
        requested = [v for v in _DEBUG_VIEWS if self.get_parameter(f'debug_{v}_enabled').value]
        limit = self.get_parameter('max_concurrent_debug_views').value
        if len(requested) > limit:
            self.get_logger().warn(
                f'同时请求的调试图像 {requested} 超过 max_concurrent_debug_views={limit}，'
                f'只算/只发前 {limit} 路：{requested[:limit]}', throttle_duration_sec=5.0)
        return requested[:limit]

    def _publish_debug_image(self, view, image_bgr, debug, header):
        if view == 'mask':
            out_bgr = cv2.cvtColor(debug['mask'], cv2.COLOR_GRAY2BGR)
        elif view == 'contours':
            out_bgr = image_bgr.copy()
            for obs, det_msg in debug['contour_results']:
                color = (0, 200, 0) if det_msg is not None else (128, 128, 128)  # 接受=绿，拒绝=灰
                cv2.drawContours(out_bgr, [obs.contour], -1, color, 2)
        elif view == 'annotated':
            out_bgr = image_bgr.copy()
            for obs, det_msg in debug['contour_results']:
                if det_msg is None:
                    continue
                cv2.circle(out_bgr, (int(obs.u), int(obs.v)), int(obs.radius_px), (0, 200, 0), 2)
                label = f'{det_msg.apparent_size:.2f}m {det_msg.confidence:.2f}'
                cv2.putText(out_bgr, label, (int(obs.u) - 30, int(obs.v) - int(obs.radius_px) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1, cv2.LINE_AA)
        else:
            return
        out_msg = self._bridge.cv2_to_imgmsg(out_bgr, encoding='bgr8')
        out_msg.header = header
        self._debug_image_publishers[view].publish(out_msg)

    def _publish_perf(self, perf):
        status = DiagnosticStatus()
        status.name = 'cuadc_perception_cv'
        status.level = DiagnosticStatus.OK
        status.values = [KeyValue(key=k, value=f'{v:.2f}') for k, v in perf.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self._perf_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CvPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
