"""传统 CV 检测算法：分割 -> 形态学去噪 -> 轮廓提取 -> 拟合筛选 -> 反投影。
纯函数（numpy 输入输出，不依赖 rclpy），可以脱离节点单测。算法选型依据见
docs/spec/05-perception-decision-interface.md；反投影/置信度/精修策略的
设计讨论记录在这个分支的 PR 描述里，不重复抄一份到代码注释。

跟决策层的契约（Detection 的 position/apparent_size/confidence 语义）见
docs/spec/05-perception-decision-interface.md，这里只管算出这三个值，不
关心 ROS 消息类型。
"""

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class CameraParams:
    fx: float
    fy: float
    cx: float
    cy: float
    mount_roll_rad: float = 0.0
    mount_pitch_rad: float = 1.5708  # nadir 惯例，见 gazebo_bringup.launch.py 同名参数
    mount_yaw_rad: float = 0.0


@dataclass
class VehiclePose:
    position_ned: Tuple[float, float, float]  # (north, east, down)
    orientation_xyzw: Tuple[float, float, float, float]  # 机体->世界四元数


@dataclass
class Detection:
    position_ned: Tuple[float, float]
    apparent_radius_m: float
    confidence: float


_MIN_RAY_DOWN_COMPONENT = 0.15  # 射线太接近水平（姿态倾斜太大）时反投影解不出合理交点，直接丢弃这一条


def detect_cylinders(image_bgr, camera: CameraParams, pose: VehiclePose,
                      cylinder_height_m: float, min_radius_m: float, max_radius_m: float,
                      morph_kernel_px: int = 5, perf: Optional[dict] = None) -> List[Detection]:
    """`perf` 传一个空 dict 进来会被填上分阶段耗时（毫秒）——感知节点用来发
    profiling 诊断信息，纯单测不用管这个参数。"""
    t0 = time.monotonic()
    mask = _segment(image_bgr)
    t1 = time.monotonic()
    mask = _denoise(mask, morph_kernel_px)
    t2 = time.monotonic()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    t3 = time.monotonic()

    body_to_world = _quaternion_to_matrix(*pose.orientation_xyzw)
    cam_to_body = _mount_rotation_matrix(camera.mount_roll_rad, camera.mount_pitch_rad, camera.mount_yaw_rad)
    cam_to_world = body_to_world @ cam_to_body

    detections = []
    for contour in contours:
        det = _fit_contour(contour, camera, pose, cam_to_world, cylinder_height_m,
                            min_radius_m, max_radius_m)
        if det is not None:
            detections.append(det)
    t4 = time.monotonic()

    if perf is not None:
        perf['segment_ms'] = (t1 - t0) * 1000.0
        perf['morphology_ms'] = (t2 - t1) * 1000.0
        perf['contours_ms'] = (t3 - t2) * 1000.0
        perf['fit_reproject_ms'] = (t4 - t3) * 1000.0
        perf['total_ms'] = (t4 - t0) * 1000.0

    return detections


def _segment(image_bgr):
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return mask


def _denoise(mask, kernel_px):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_px, kernel_px))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def _fit_contour(contour, camera, pose, cam_to_world, cylinder_height_m,
                  min_radius_m, max_radius_m) -> Optional[Detection]:
    area = cv2.contourArea(contour)
    if area <= 0:
        return None
    (u, v), enclosing_radius_px = cv2.minEnclosingCircle(contour)
    if enclosing_radius_px <= 0:
        return None
    # circularity（形状是否规则、置信度用）用外接圆半径算——外接圆能不能
    # 撑满是"形状规不规则"的量度。实际物理尺寸不能用外接圆半径算：斜视时
    # 只看得到筒的一小段新月形侧影，外接圆为了包住这个细长月牙形状，
    # 半径会被撑得比真实筒径大很多（实测：一帧真实图像外接圆半径对应
    # 换算出0.34m，远超所有筒的真实半径范围）。等效面积半径对形状拉长
    # 不敏感，斜视时只是温和高估，不会离谱，用它做尺寸过滤和实际半径估计。
    area_equiv_radius_px = math.sqrt(area / math.pi)
    circularity = area / (math.pi * enclosing_radius_px ** 2)

    world_xy, range_m = _reproject_to_cylinder_top(u, v, camera, pose, cam_to_world, cylinder_height_m)
    if world_xy is None:
        return None

    real_radius_m = area_equiv_radius_px * range_m / camera.fx
    if not (min_radius_m <= real_radius_m <= max_radius_m):
        return None

    return Detection(position_ned=world_xy, apparent_radius_m=real_radius_m,
                      confidence=min(circularity, 1.0))


def _reproject_to_cylinder_top(u, v, camera: CameraParams, pose: VehiclePose,
                                cam_to_world, cylinder_height_m):
    """像素 -> 世界坐标：像素反投影成相机系射线，转到世界系，跟筒顶所在
    平面（NED down = -cylinder_height_m，不是地面 down=0）求交——检测到
    的色块主要反映筒的顶面，跟地面求交会引入正比于视线倾角的系统性偏差。
    返回 (world_xy, 斜距)，射线太接近水平（姿态倾斜太大）时返回
    (None, None)。
    """
    d_cam = np.array([(u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy, 1.0])
    d_world = cam_to_world @ d_cam

    origin_down = pose.position_ned[2]
    if d_world[2] < _MIN_RAY_DOWN_COMPONENT:
        return None, None
    t = (-cylinder_height_m - origin_down) / d_world[2]
    if t <= 0:
        return None, None

    world_north = pose.position_ned[0] + t * d_world[0]
    world_east = pose.position_ned[1] + t * d_world[1]
    range_m = t * float(np.linalg.norm(d_cam))
    return (world_north, world_east), range_m


def _quaternion_to_matrix(x, y, z, w):
    """机体->世界旋转矩阵，标准四元数公式，跟 decision_node.py 里
    `_roll_pitch_from_quaternion` 提取 roll/pitch 用的是同一个约定
    （ArduPilot/MAVLink 姿态四元数）。"""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _mount_rotation_matrix(roll, pitch, yaw):
    """相机->机体固定挂载旋转。注意：这里的 pitch 符号约定是云台
    `/gimbal/cmd_pitch` 话题自己的约定（+1.5708 是 nadir，实测验证，
    见 gazebo_bringup.launch.py），跟机体姿态四元数的俯仰符号约定不是
    同一件事，两者不能混用同一个旋转公式——机体姿态用标准四元数公式
    （见 `_quaternion_to_matrix`），这里单独用云台自己的约定。默认挂载
    （roll=yaw=0, pitch=+1.5708）已经用合成图像单测验证过内部自洽，
    但这个约定本身（尤其 roll/yaw 非零时）还没有拿真实 Gazebo 图像
    实测确认，见节点验证计划。
    """
    cp, sp = math.cos(pitch), math.sin(pitch)
    ry = np.array([[cp, 0, -sp], [0, 1, 0], [sp, 0, cp]])
    cr, sr = math.cos(roll), math.sin(roll)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    # 相机自身光轴约定（x-right, y-down, z-forward）在零挂载角时对齐机体
    # 前向（body_x=cam_z, body_y=cam_x, body_z=cam_y）
    base = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    return rz @ ry @ rx @ base
