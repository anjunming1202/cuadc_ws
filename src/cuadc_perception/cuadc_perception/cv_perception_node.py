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

import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header

from cuadc_perception import cv_pipeline
from cuadc_perception_msgs.msg import Detection, DetectionArray

_DETECTIONS_TOPIC = '/perception/detections'
_IMAGE_TOPIC = '/camera/image_raw'
_LOCAL_POSITION_TOPIC = '/cuadc_interface/telemetry/local_position'
_ATTITUDE_TOPIC = '/cuadc_interface/telemetry/attitude'


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

        self._bridge = CvBridge()
        self._latest_position_ned = None  # (north, east, down)
        self._latest_orientation_xyzw = None

        self._publisher = self.create_publisher(DetectionArray, _DETECTIONS_TOPIC, 10)
        self._perf_publisher = self.create_publisher(DiagnosticArray, '~/perf', 10)
        self.create_subscription(Image, _IMAGE_TOPIC, self._on_image, 10)
        self.create_subscription(PoseStamped, _LOCAL_POSITION_TOPIC, self._on_local_position, 10)
        self.create_subscription(Imu, _ATTITUDE_TOPIC, self._on_attitude, 10)

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

        perf = {}
        detections = cv_pipeline.detect_cylinders(
            image_bgr, camera, pose,
            cylinder_height_m=self.get_parameter('cylinder_height_m').value,
            min_radius_m=self.get_parameter('min_radius_m').value,
            max_radius_m=self.get_parameter('max_radius_m').value,
            morph_kernel_px=self.get_parameter('morph_kernel_px').value,
            perf=perf,
        )

        out = DetectionArray()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.detections = [self._to_msg(d) for d in detections]
        self._publisher.publish(out)

        self._publish_perf(perf)

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
