"""真实感知节点：订阅相机图像 + 遥测（位置/姿态），跑 cv_pipeline 里的
传统 CV 算法，发布跟桩节点同一份契约（docs/spec/05-perception-decision-interface.md）
的 DetectionArray——决策层不需要区分对面是桩还是这个节点。

这里只做 ROS2 接线（订阅/参数/发布），算法本身在 cv_pipeline.py（纯
函数，不依赖 rclpy，可以脱离节点单测），跟 mission_logic.py/decision_node.py
的分层原则一致。

按图像帧率处理（收到一帧处理一帧），不像桩节点那样用固定频率定时器
——桩节点没有真实数据源触发，只能自己定频率；这里数据源就是相机本身
的帧率（sim 里是 10Hz，见 gazebo_bringup.launch.py 的相机 update_rate）。
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header, String

from cuadc_perception import cv_pipeline
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
        self.declare_parameter('morph_kernel_px', 5)
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

        camera = cv_pipeline.CameraParams(
            fx=self.get_parameter('fx').value,
            fy=self.get_parameter('fy').value,
            cx=self.get_parameter('cx').value,
            cy=self.get_parameter('cy').value,
            mount_roll_rad=self.get_parameter('camera_fixed_roll_rad').value,
            mount_pitch_rad=self.get_parameter('camera_fixed_pitch_rad').value,
            mount_yaw_rad=self.get_parameter('camera_fixed_yaw_rad').value,
        )
        pose = cv_pipeline.VehiclePose(
            position_ned=self._latest_position_ned,
            orientation_xyzw=self._latest_orientation_xyzw,
        )

        active_views = self._active_debug_views()
        perf = {}
        debug = {} if active_views else None
        detections = cv_pipeline.detect_cylinders(
            image_bgr, camera, pose,
            cylinder_height_m=self.get_parameter('cylinder_height_m').value,
            min_radius_m=self.get_parameter('min_radius_m').value,
            max_radius_m=self.get_parameter('max_radius_m').value,
            morph_kernel_px=self.get_parameter('morph_kernel_px').value,
            perf=perf,
            debug=debug,
        )

        for view in active_views:
            self._publish_debug_image(view, image_bgr, debug, msg.header)
        if self._recon_zone_active:
            self._recon_image_publisher.publish(msg)

        out = DetectionArray()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.detections = [self._to_msg(d) for d in detections]
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
            for contour, det in debug['contour_results']:
                color = (0, 200, 0) if det is not None else (128, 128, 128)  # 接受=绿，拒绝=灰
                cv2.drawContours(out_bgr, [contour], -1, color, 2)
        elif view == 'annotated':
            out_bgr = image_bgr.copy()
            for contour, det in debug['contour_results']:
                if det is None:
                    continue
                (u, v), r_px = cv2.minEnclosingCircle(contour)  # 只用于画图定位，不是检测算法的一部分
                cv2.circle(out_bgr, (int(u), int(v)), int(r_px), (0, 200, 0), 2)
                label = f'{det.apparent_radius_m:.2f}m {det.confidence:.2f}'
                cv2.putText(out_bgr, label, (int(u) - 30, int(v) - int(r_px) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1, cv2.LINE_AA)
        else:
            return
        out_msg = self._bridge.cv2_to_imgmsg(out_bgr, encoding='bgr8')
        out_msg.header = header
        self._debug_image_publishers[view].publish(out_msg)

    @staticmethod
    def _to_msg(det: cv_pipeline.Detection) -> Detection:
        msg = Detection()
        msg.target_type = 'cylinder'
        msg.position.x = det.position_ned[0]
        msg.position.y = det.position_ned[1]
        msg.position.z = 0.0
        msg.apparent_size = det.apparent_radius_m
        msg.confidence = det.confidence
        return msg

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
