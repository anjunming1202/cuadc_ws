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
  同时把标注预览编码推到本地 `preview_port`（onboard 转发到地面）。
"""

import math
import threading
import time
from collections import deque

import cv2
import cuadc_physical
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header, String

from cuadc_perception import detectors, geometry
from cuadc_perception.frame_sources import DEFAULT_RAW_SHM, DEFAULT_STAMP_PORT, ShmFrameSource
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


def _stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _interp_vec(buf, t):
    """在按时间升序的 (t, vec) 缓冲里线性插值 vec@t；空->None；t 超界->钳到最近端。"""
    if not buf:
        return None
    if t <= buf[0][0]:
        return buf[0][1]
    if t >= buf[-1][0]:
        return buf[-1][1]
    prev = buf[0]
    for cur in buf:
        if cur[0] >= t:
            t0, v0 = prev
            t1, v1 = cur
            a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            return tuple(v0[k] + a * (v1[k] - v0[k]) for k in range(len(v0)))
        prev = cur
    return buf[-1][1]


def _interp_quat(buf, t):
    """四元数按时间 nlerp@t（符号对齐 + 归一化，小角近似 slerp，够用）。"""
    if not buf:
        return None
    if t <= buf[0][0]:
        return buf[0][1]
    if t >= buf[-1][0]:
        return buf[-1][1]
    prev = buf[0]
    for cur in buf:
        if cur[0] >= t:
            t0, q0 = prev
            t1, q1 = cur
            a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            d = sum(q0[i] * q1[i] for i in range(4))
            if d < 0.0:
                q1 = tuple(-c for c in q1)
            q = tuple(q0[i] + a * (q1[i] - q0[i]) for i in range(4))
            n = math.sqrt(sum(c * c for c in q)) or 1.0
            return tuple(c / n for c in q)
        prev = cur
    return buf[-1][1]


class CvPerceptionNode(Node):

    def __init__(self):
        super().__init__('cuadc_perception_cv')

        # 相机内参 + 挂载角按 profile 从 camera.<profile>.yaml 读(见 geometry.load_camera_params)。
        # 默认取规则书投放筒高（cuadc_physical/targets.yaml）；仍可用参数临时覆盖。
        self.declare_parameter('cylinder_height_m', cuadc_physical.load_targets().drop_cylinder_height_m)
        self.declare_parameter('min_radius_m', 0.05)
        self.declare_parameter('max_radius_m', 0.18)
        self.declare_parameter('detector', detectors.ACTIVE_DEFAULT)
        self.declare_parameter('profile', 'sim')
        for view in _DEBUG_VIEWS:
            self.declare_parameter(f'debug_{view}_enabled', False)
        self.declare_parameter('max_concurrent_debug_views', 4)

        # 取帧源:sim=ros(订阅 image_raw)、real=shm(读原始帧,图不进 ROS)。
        self.declare_parameter('frame_source', 'ros')
        # capture=按拍照时刻插值(生产默认) / latest=处理时刻最新——仅 A/B 对照与
        # 真机拍照戳异常兜底,启用即告警。
        self.declare_parameter('pose_at', 'capture')
        self.declare_parameter('raw_shm', DEFAULT_RAW_SHM)
        self.declare_parameter('src_width', 1920)
        self.declare_parameter('src_height', 1080)
        self.declare_parameter('src_fps', 10)
        self.declare_parameter('cv_width', 960)    # shm 源出帧/检测/预览的分辨率
        self.declare_parameter('cv_height', 540)
        # shm 源的帧时间戳：相机服务旁路发来的拍照时刻(SensorTimestamp)；配不上时退回到达时刻并告警。
        self.declare_parameter('capture_stamps', True)
        self.declare_parameter('stamp_port', DEFAULT_STAMP_PORT)
        # RTP 标注预览(真机给地面看的 CV 框):默认关,真机 launch 打开。
        self.declare_parameter('preview_enabled', False)
        self.declare_parameter('preview_port', DEFAULT_PREVIEW_PORT)
        self.declare_parameter('preview_bitrate', 1000)
        # 逐帧检测判定链日志(px 半径→反投影距离→物理半径→接受/拒因),节流打印。
        # 默认关;运行时可 `ros2 param set /cuadc_perception_cv detection_log_enabled true` 开。
        self.declare_parameter('detection_log_enabled', False)

        detector_name = self.get_parameter('detector').value
        profile = self.get_parameter('profile').value
        detector = detectors.DETECTORS[detector_name]
        cfg = detector.load_config(profile)
        self.get_logger().info(f'[DETECTOR] {detector_name} (profile={profile})')

        camera = geometry.load_camera_params(profile)
        self._core = PerceptionCore(
            detector, cfg, camera,
            cylinder_height_m=self.get_parameter('cylinder_height_m').value,
            min_radius_m=self.get_parameter('min_radius_m').value,
            max_radius_m=self.get_parameter('max_radius_m').value,
        )

        self._bridge = CvBridge()
        # 位姿时间缓冲：反投影按图像拍照时刻插值取位姿(否则运动/延迟时出散点,
        # ADR-0010 B)。前提图像与遥测同时钟(sim=/clock, real=Pi 时钟)。
        self._pos_buf = deque(maxlen=300)  # (t_sec, (x,y,z)); 300样本≈3-6s,覆盖链路延迟
        self._att_buf = deque(maxlen=300)  # (t_sec, (x,y,z,w))
        self._clock_mismatch_warned = False
        self._pose_at_latest = self.get_parameter('pose_at').value == 'latest'
        if self._pose_at_latest:
            self.get_logger().warn('pose_at=latest: TEST ONLY -- capture-time alignment disabled')
        self._latest_position = None
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
        self._stamp_stats, self._stamp_ages, self._stamp_log_t = [0, 0], [], time.monotonic()
        source = self.get_parameter('frame_source').value
        if source == 'shm':
            self._shm_source = ShmFrameSource(
                raw_shm=self.get_parameter('raw_shm').value,
                src_w=self.get_parameter('src_width').value,
                src_h=self.get_parameter('src_height').value,
                src_fps=self.get_parameter('src_fps').value,
                out_w=self.get_parameter('cv_width').value,
                out_h=self.get_parameter('cv_height').value,
                stamp_port=(self.get_parameter('stamp_port').value
                            if self.get_parameter('capture_stamps').value else 0),
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
        now = self.get_clock().now()
        captured = self._shm_source.last_capture_ns()
        if captured is None:
            header.stamp = now.to_msg()  # 退回到达时刻（比拍照晚 ~30ms）
            self._stamp_stats[1] += 1
        else:
            age_ns = time.monotonic_ns() - captured
            header.stamp = (now - Duration(nanoseconds=age_ns)).to_msg()
            self._stamp_stats[0] += 1
            self._stamp_ages.append(age_ns / 1e6)
        self._log_stamp_stats()
        self._process_frame(frame_bgr, header, ros_image_msg=None)

    def _log_stamp_stats(self):
        """每 10s 一条：拍照时刻配对率 + 拍照->打戳时延（录进 bag，现场延迟有没有漂一看便知）。"""
        t = time.monotonic()
        if t - self._stamp_log_t < 10.0:
            return
        matched, fallback = self._stamp_stats
        ages = sorted(self._stamp_ages)
        lat = f'capture->stamp p50 {ages[len(ages) // 2]:.0f} / max {ages[-1]:.0f} ms' if ages else 'no capture stamps'
        log = self.get_logger().warn if fallback and not matched else self.get_logger().info
        log(f'[CAPTURE_STAMP] matched {matched}/{matched + fallback} frames, {lat}')
        self._stamp_log_t, self._stamp_stats, self._stamp_ages = t, [0, 0], []

    # ---- 遥测/状态 ----
    def _on_mission_state(self, msg):
        self._recon_zone_active = (msg.data == _RECON_ZONE_STATE)

    def _on_local_position(self, msg):
        p = msg.pose.position
        self._latest_position = (p.x, p.y, p.z)
        self._pos_buf.append((_stamp_sec(msg.header.stamp), (p.x, p.y, p.z)))

    def _on_attitude(self, msg):
        q = msg.orientation
        self._latest_orientation_xyzw = (q.x, q.y, q.z, q.w)
        self._att_buf.append((_stamp_sec(msg.header.stamp), (q.x, q.y, q.z, q.w)))

    def _pose_at(self, t_capture):
        """按图像捕获时刻在位姿缓冲里插值。缓冲空 -> 用最新(退化=旧行为)；t 超出缓冲
        范围 -> 钳到最近端(位姿比图像晚到时用最新、比图像早时用最旧)。
        时钟域失配守卫: 拍照戳与最新遥测戳差 >5s = 图像/遥测不在同一时钟(sim 忘开
        use_sim_time 之类) -> 告警一次并退化用最新位姿,绝不静默钳到远古位姿。"""
        if self._pose_at_latest:  # 证据链 A/B 对照旋钮,见参数声明
            if self._latest_position is None or self._latest_orientation_xyzw is None:
                return None
            return geometry.VehiclePose(position=self._latest_position,
                                        orientation_xyzw=self._latest_orientation_xyzw)
        if self._att_buf and abs(t_capture - self._att_buf[-1][0]) > 5.0:
            if not self._clock_mismatch_warned:
                self._clock_mismatch_warned = True
                self.get_logger().warn(
                    f'image stamp {t_capture:.1f} vs telemetry {self._att_buf[-1][0]:.1f}: '
                    'clock domains differ (sim without use_sim_time?) -- falling back to '
                    'latest pose, capture-time alignment DISABLED')
            if self._latest_position is None or self._latest_orientation_xyzw is None:
                return None
            return geometry.VehiclePose(position=self._latest_position,
                                        orientation_xyzw=self._latest_orientation_xyzw)
        pos = _interp_vec(self._pos_buf, t_capture)
        quat = _interp_quat(self._att_buf, t_capture)
        if pos is None or quat is None:
            if self._latest_position is None or self._latest_orientation_xyzw is None:
                return None
            return geometry.VehiclePose(position=self._latest_position,
                                        orientation_xyzw=self._latest_orientation_xyzw)
        return geometry.VehiclePose(position=pos, orientation_xyzw=quat)

    # ---- 主处理:检测一次,结果扇出 ----
    def _process_frame(self, frame_bgr, header, ros_image_msg=None):
        # 用图像捕获时刻(header.stamp)插值取位姿——不是处理时刻的最新位姿(见 ADR-0010 B)。
        pose = self._pose_at(_stamp_sec(header.stamp))
        if pose is None:
            return  # 没位姿不知道飞机在哪,没法反投影
        active_views = self._active_debug_views()
        want_debug = bool(active_views) or self._preview is not None
        result = self._core.process(frame_bgr, pose, want_debug=want_debug)

        # sink 1: DetectionArray(ROS,给决策)。stamp=拍照时刻、vehicle_pose=拍照时刻位姿
        # (与 bearing 同刻,契约见 DetectionArray.msg / 05 文档)。
        out = DetectionArray()
        out.header = Header()
        out.header.stamp = header.stamp
        out.vehicle_pose.position.x = pose.position[0]
        out.vehicle_pose.position.y = pose.position[1]
        out.vehicle_pose.position.z = pose.position[2]
        (out.vehicle_pose.orientation.x, out.vehicle_pose.orientation.y,
         out.vehicle_pose.orientation.z, out.vehicle_pose.orientation.w) = pose.orientation_xyzw
        out.detections = [self._to_msg(d) for d in result.detections]
        self._publisher.publish(out)
        self._publish_perf(result.perf)
        if self.get_parameter('detection_log_enabled').value:
            self._log_detection_chain(result)

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

    def _log_detection_chain(self, result):
        """把这一帧每个候选的判定链打成一行(节流~1s):px 半径→反投影距离→物理半径→接受/拒因。
        地面单点调参用;radius_m 超出 [min,max] 会显示 too_small/too_big(地面高度≈0 时常见)。"""
        parts = []
        for i, cr in enumerate(result.contour_results):
            rng = f'{cr.range_m:.2f}m' if cr.range_m is not None else 'n/a'
            r_m = f'{cr.radius_m:.3f}m' if cr.radius_m is not None else 'n/a'
            if cr.detection is not None:
                d = cr.detection
                parts.append(f'#{i} px_r={cr.observation.radius_px:.0f} rng={rng} r={r_m} '
                             f'ACC conf={d.confidence:.2f} world=({d.x:.1f},{d.y:.1f})')
            else:
                parts.append(f'#{i} px_r={cr.observation.radius_px:.0f} rng={rng} r={r_m} '
                             f'REJ({cr.reject_reason})')
        detail = ' | '.join(parts) if parts else '(no contours)'
        self.get_logger().info(
            f'[DET] obs={len(result.observations)} acc={len(result.detections)} | {detail}',
            throttle_duration_sec=1.0)

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
            for cr in debug['contour_results']:
                obs, det = cr.observation, cr.detection
                color = (0, 200, 0) if det is not None else (128, 128, 128)
                cv2.drawContours(out_bgr, [obs.contour], -1, color, 2)
        elif view == 'annotated':
            out_bgr = image_bgr.copy()
            for cr in debug['contour_results']:
                obs, det = cr.observation, cr.detection
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
