"""真实感知节点：**薄 ROS 壳**——选取帧源（sim=`/camera/image_raw`；real=相机
服务的原始 NV12 shm，图像**不进 ROS**）、跑 rclpy-free 的 `PerceptionCore`、把结果
扇出到多个 sink：DetectionArray（ROS，给决策，契约不变）、sim 的 debug Image（给
cuadc_viz）、real 的 RTP 标注预览（给地面）。算法在 `detectors/`、编排在
`perception_core`、取帧/预览在 `frame_sources`/`sinks`——都不依赖 rclpy。
设计见 docs/spec/adr/0008-perception-raw-frame-multi-sink.md、契约见
docs/spec/05-perception-decision-interface.md。

按帧处理（收一帧算一帧）。取帧源:
- `frame_source=ros`（仿真）:订阅 `/camera/image_raw`（Gazebo 发），图经 ROS。
- `frame_source=shm`（真机）:从 `raw_shm` 直接取原始帧,零解码,**图不进 ROS**;
  同时把标注预览编码推到本地 `preview_port`（cv-forward 转地面）。
"""

import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header, String

from cuadc_perception import detectors, geometry
from cuadc_perception.frame_sources import DEFAULT_RAW_SHM, ShmFrameSource
from cuadc_perception.perception_core import PerceptionCore
from cuadc_perception.sinks import DEFAULT_PREVIEW_PORT, RtpPreviewSink, draw_overlay
from cuadc_perception_msgs.msg import Detection, DetectionArray

_DETECTIONS_TOPIC = '/perception/detections'
_IMAGE_TOPIC = '/camera/image_raw'
_LOCAL_POSITION_TOPIC = '/cuadc_interface/telemetry/local_position'
_ATTITUDE_TOPIC = '/cuadc_interface/telemetry/attitude'
_MISSION_STATE_TOPIC = '/cuadc_decision/mission_state'

_RECON_ZONE_STATE = 'SURVEY_RECON_ZONE'
_DEBUG_VIEWS = ('annotated', 'contours', 'mask')


class CvPerceptionNode(Node):

    def __init__(self):
        super().__init__('cuadc_perception_cv')

        # 相机内参:sim 默认值(640x480@fov2.0);真机传该机在**CV 处理分辨率**下的
        # 标定值(注意 shm 源出的是 cv_width×cv_height,内参要对应这个分辨率,不是
        # 全分辨率标定原值)。见 ADR-0008 "前置条件"。
        self.declare_parameter('fx', 205.5)
        self.declare_parameter('fy', 205.5)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('camera_fixed_roll_rad', 0.0)
        self.declare_parameter('camera_fixed_pitch_rad', 1.5708)
        self.declare_parameter('camera_fixed_yaw_rad', 0.0)
        self.declare_parameter('cylinder_height_m', 0.3)
        self.declare_parameter('min_radius_m', 0.05)
        self.declare_parameter('max_radius_m', 0.18)
        self.declare_parameter('detector', detectors.ACTIVE_DEFAULT)
        self.declare_parameter('profile', 'sim')
        for view in _DEBUG_VIEWS:
            self.declare_parameter(f'debug_{view}_enabled', False)
        self.declare_parameter('max_concurrent_debug_views', 4)

        # 取帧源:sim=ros(订阅 image_raw)、real=shm(读原始帧,图不进 ROS)。
        self.declare_parameter('frame_source', 'ros')
        self.declare_parameter('raw_shm', DEFAULT_RAW_SHM)
        self.declare_parameter('src_width', 1920)
        self.declare_parameter('src_height', 1080)
        self.declare_parameter('src_fps', 10)
        self.declare_parameter('cv_width', 960)    # shm 源出帧/检测/预览的分辨率
        self.declare_parameter('cv_height', 540)
        # RTP 标注预览(真机给地面看的 CV 框):默认关,真机 launch 打开。
        self.declare_parameter('preview_enabled', False)
        self.declare_parameter('preview_port', DEFAULT_PREVIEW_PORT)
        self.declare_parameter('preview_bitrate', 1000)

        detector_name = self.get_parameter('detector').value
        profile = self.get_parameter('profile').value
        detector = detectors.DETECTORS[detector_name]
        cfg = detector.load_config(profile)
        self.get_logger().info(f'[DETECTOR] {detector_name} (profile={profile})')

        camera = geometry.CameraParams(
            fx=self.get_parameter('fx').value, fy=self.get_parameter('fy').value,
            cx=self.get_parameter('cx').value, cy=self.get_parameter('cy').value,
            mount_roll_rad=self.get_parameter('camera_fixed_roll_rad').value,
            mount_pitch_rad=self.get_parameter('camera_fixed_pitch_rad').value,
            mount_yaw_rad=self.get_parameter('camera_fixed_yaw_rad').value,
        )
        self._core = PerceptionCore(
            detector, cfg, camera,
            cylinder_height_m=self.get_parameter('cylinder_height_m').value,
            min_radius_m=self.get_parameter('min_radius_m').value,
            max_radius_m=self.get_parameter('max_radius_m').value,
        )

        self._bridge = CvBridge()
        self._latest_position_ned = None
        self._latest_orientation_xyzw = None
        self._recon_zone_active = False
        self._frame_idx = 0
        self._ema_ms = 0.0

        self._publisher = self.create_publisher(DetectionArray, _DETECTIONS_TOPIC, 10)
        self._perf_publisher = self.create_publisher(DiagnosticArray, '~/perf', 10)
        self._debug_image_publishers = {
            view: self.create_publisher(Image, f'~/debug/{view}', 10) for view in _DEBUG_VIEWS
        }
        self._recon_image_publisher = self.create_publisher(Image, '~/recon_image', 10)

        self.create_subscription(PoseStamped, _LOCAL_POSITION_TOPIC, self._on_local_position, 10)
        self.create_subscription(Imu, _ATTITUDE_TOPIC, self._on_attitude, 10)
        self.create_subscription(String, _MISSION_STATE_TOPIC, self._on_mission_state, 10)

        # RTP 预览 sink(真机)
        self._preview = None
        if self.get_parameter('preview_enabled').value:
            self._preview = RtpPreviewSink(
                port=self.get_parameter('preview_port').value,
                width=self.get_parameter('cv_width').value,
                height=self.get_parameter('cv_height').value,
                fps=self.get_parameter('src_fps').value,
                bitrate=self.get_parameter('preview_bitrate').value,
            )
            self._preview.start()

        # 取帧源接线
        self._shm_source = None
        source = self.get_parameter('frame_source').value
        if source == 'shm':
            self._shm_source = ShmFrameSource(
                raw_shm=self.get_parameter('raw_shm').value,
                src_w=self.get_parameter('src_width').value,
                src_h=self.get_parameter('src_height').value,
                src_fps=self.get_parameter('src_fps').value,
                out_w=self.get_parameter('cv_width').value,
                out_h=self.get_parameter('cv_height').value,
            )
            if not self._shm_source.start(self._on_shm_frame):
                self.get_logger().error('shm 取帧源启动失败(相机服务在跑吗?)')
        else:
            self.create_subscription(Image, _IMAGE_TOPIC, self._on_ros_image, 10)

    # ---- 取帧回调 ----
    def _on_ros_image(self, msg):
        frame_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._process_frame(frame_bgr, msg.header, ros_image_msg=msg)

    def _on_shm_frame(self, frame_bgr):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        self._process_frame(frame_bgr, header, ros_image_msg=None)

    # ---- 遥测/状态 ----
    def _on_mission_state(self, msg):
        self._recon_zone_active = (msg.data == _RECON_ZONE_STATE)

    def _on_local_position(self, msg):
        self._latest_position_ned = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _on_attitude(self, msg):
        q = msg.orientation
        self._latest_orientation_xyzw = (q.x, q.y, q.z, q.w)

    # ---- 主处理:检测一次,结果扇出 ----
    def _process_frame(self, frame_bgr, header, ros_image_msg=None):
        position = self._latest_position_ned
        orientation = self._latest_orientation_xyzw
        if position is None or orientation is None:
            return  # 没位姿不知道飞机在哪,没法反投影

        pose = geometry.VehiclePose(position_ned=position, orientation_xyzw=orientation)
        active_views = self._active_debug_views()
        want_debug = bool(active_views) or self._preview is not None
        result = self._core.process(frame_bgr, pose, want_debug=want_debug)

        # sink 1: DetectionArray(ROS,给决策)
        out = DetectionArray()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.detections = [self._to_msg(d) for d in result.detections]
        self._publisher.publish(out)
        self._publish_perf(result.perf)

        # sink 2: sim 的 debug Image(给 cuadc_viz)
        for view in active_views:
            self._publish_debug_image(view, frame_bgr, result.debug, header)
        if self._recon_zone_active and ros_image_msg is not None:
            self._recon_image_publisher.publish(ros_image_msg)  # 真机侦察走视频下行,不发这个

        # sink 3: real 的 RTP 标注预览(给地面)
        if self._preview is not None:
            self._frame_idx += 1
            self._ema_ms = (result.perf.get('total_ms', 0.0) if self._ema_ms <= 0
                            else 0.8 * self._ema_ms + 0.2 * result.perf.get('total_ms', 0.0))
            annotated = frame_bgr.copy()
            draw_overlay(annotated, result, self._frame_idx, self._ema_ms)
            self._preview.push(annotated)

    @staticmethod
    def _to_msg(det):
        m = Detection()
        m.target_type = det.target_type
        m.position.x, m.position.y, m.position.z = det.x, det.y, det.z
        m.bearing_x, m.bearing_y = det.bearing_x, det.bearing_y
        m.apparent_size = det.apparent_size
        m.confidence = det.confidence
        return m

    def _active_debug_views(self):
        requested = [v for v in _DEBUG_VIEWS if self.get_parameter(f'debug_{v}_enabled').value]
        limit = self.get_parameter('max_concurrent_debug_views').value
        if len(requested) > limit:
            self.get_logger().warn(
                f'同时请求的调试图像 {requested} 超过 max_concurrent_debug_views={limit}，'
                f'只算/只发前 {limit} 路：{requested[:limit]}', throttle_duration_sec=5.0)
        return requested[:limit]

    def _publish_debug_image(self, view, image_bgr, debug, header):
        if debug is None:
            return
        if view == 'mask':
            out_bgr = cv2.cvtColor(debug['mask'], cv2.COLOR_GRAY2BGR)
        elif view == 'contours':
            out_bgr = image_bgr.copy()
            for obs, det in debug['contour_results']:
                color = (0, 200, 0) if det is not None else (128, 128, 128)
                cv2.drawContours(out_bgr, [obs.contour], -1, color, 2)
        elif view == 'annotated':
            out_bgr = image_bgr.copy()
            for obs, det in debug['contour_results']:
                if det is None:
                    continue
                cv2.circle(out_bgr, (int(obs.u), int(obs.v)), int(obs.radius_px), (0, 200, 0), 2)
                label = f'{det.apparent_size:.2f}m {det.confidence:.2f}'
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

    def destroy_node(self):
        if self._shm_source is not None:
            self._shm_source.stop()
        if self._preview is not None:
            self._preview.stop()
        super().destroy_node()


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
