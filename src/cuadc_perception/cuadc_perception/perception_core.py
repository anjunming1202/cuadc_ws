"""感知流水线核心：frame(+pose) → 检测器 → 反投影/物理半径过滤 → 结果。

**不依赖 rclpy**（延续 detectors/ "算法不依赖 rclpy" 的原则到整条流水线，可脱离
机器人单测）。ROS 只在取帧源 / 结果 sink / 薄壳节点里出现，见
docs/spec/adr/0008-perception-raw-frame-multi-sink.md。

`pose=None` 是**预览模式**：只出图像空间观测（画框用），不反投影、不出世界检测
——供相机自检/调参的无-ROS 预览复用同一套检测。
"""
from __future__ import annotations

import time
from dataclasses import dataclass

from cuadc_perception import geometry


@dataclass
class WorldDetection:
    """一个被接受的目标（世界坐标 + 归一化方位 + 物理尺寸）。纯数据，不是 ROS
    消息——ros_detection sink 负责映射成 cuadc_perception_msgs/Detection。"""
    target_type: str
    x: float
    y: float
    z: float
    bearing_x: float
    bearing_y: float
    apparent_size: float
    confidence: float


@dataclass
class ContourResult:
    """一个候选轮廓的完整判定链：图像观测 → 反投影 → 物理尺寸门。detection 非 None=通过；
    range_m/radius_m 是反投影中间量（预览模式无 pose、或反投影失败时为 None），reject_reason
    空串=接受，否则 'no_pose'/'reproject_fail'/'too_small'/'too_big'。供画框/调试/日志复用。"""
    observation: object                     # ContourObservation
    detection: "WorldDetection | None"
    range_m: float | None = None            # 反投影到筒顶的距离
    radius_m: float | None = None           # 反推的物理半径
    reject_reason: str = ''


@dataclass
class PerceptionResult:
    observations: list          # ContourObservation：图像空间观测（永远有）
    detections: list            # WorldDetection：世界检测（无 pose 时为空）
    contour_results: list       # ContourResult：逐轮廓判定链（含中间量/拒因）
    perf: dict
    debug: dict | None          # 检测器 debug（mask 等）+ 'contour_results'；未请求为 None


class PerceptionCore:
    """持检测器 + 配置 + 相机内参 + 物理半径门；`process(frame, pose)` 出结果。"""

    def __init__(self, detector, cfg, camera: geometry.CameraParams | None = None,
                 cylinder_height_m: float = 0.0, min_radius_m: float = 0.0,
                 max_radius_m: float = 0.0) -> None:
        # camera/半径门只在有 pose(反投影)时用;预览模式(pose=None)可全省。
        self._detector = detector
        self._cfg = cfg
        self._camera = camera
        self._cylinder_height_m = cylinder_height_m
        self._min_radius_m = min_radius_m
        self._max_radius_m = max_radius_m

    @property
    def camera(self) -> geometry.CameraParams:
        return self._camera

    def process(self, frame_bgr, pose: geometry.VehiclePose | None = None,
                want_debug: bool = False) -> PerceptionResult:
        perf: dict = {}
        debug: dict | None = {} if want_debug else None
        observations = self._detector.detect_cylinders_pixels(
            frame_bgr, self._cfg, perf=perf, debug=debug)

        detections: list[WorldDetection] = []
        contour_results: list = []
        if pose is not None:
            cam_to_world = geometry.cam_to_world_matrix(self._camera, pose)
            cam_origin = geometry.camera_origin(self._camera, pose)
            t_reproject = time.monotonic()
            for obs in observations:
                world_xy, range_m = geometry.reproject_to_cylinder_top(
                    obs.u, obs.v, self._camera, cam_origin, cam_to_world, self._cylinder_height_m)
                det = None
                radius_m = None
                reason = ''
                if world_xy is None:
                    range_m = None
                    reason = 'reproject_fail'
                else:
                    radius_m = geometry.transverse_radius_m(
                        obs.contour, obs.u, obs.v, self._camera, cam_to_world, range_m)
                    if radius_m is None:  # 轮廓点太少：退回面积等效半径
                        radius_m = obs.radius_px * range_m / self._camera.fx
                    if radius_m < self._min_radius_m:
                        reason = 'too_small'
                    elif radius_m > self._max_radius_m:
                        reason = 'too_big'
                    else:
                        bearing_x, bearing_y = geometry.normalize_bearing(
                            obs.u, obs.v, self._camera)
                        det = WorldDetection(
                            target_type='cylinder',
                            x=float(world_xy[0]), y=float(world_xy[1]), z=0.0,
                            bearing_x=float(bearing_x), bearing_y=float(bearing_y),
                            apparent_size=float(radius_m),
                            confidence=float(min(obs.circularity, 1.0)),
                        )
                        detections.append(det)
                contour_results.append(ContourResult(obs, det, range_m, radius_m, reason))
            reproject_ms = (time.monotonic() - t_reproject) * 1000.0
            if perf:
                perf['fit_reproject_ms'] = perf.pop('fit_ms', 0.0) + reproject_ms
                perf['total_ms'] = (perf.get('segment_ms', 0.0) + perf.get('morphology_ms', 0.0)
                                    + perf.get('contours_ms', 0.0) + perf['fit_reproject_ms'])
        else:
            contour_results = [ContourResult(obs, None, reject_reason='no_pose') for obs in observations]
            if perf:  # 预览模式无反投影,总时长 = 检测器各段之和(overlay 显示用)
                perf['total_ms'] = sum(v for k, v in perf.items() if k.endswith('_ms'))

        if debug is not None:
            debug['contour_results'] = contour_results
        return PerceptionResult(observations, detections, contour_results, perf, debug)
