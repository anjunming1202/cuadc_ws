"""共享几何助手：像素 <-> 世界坐标 / 归一化方位。检测器（detectors/ 里的）
保持纯图像空间、不碰这里；感知节点在检测器出了像素结果之后，用这里的函数
补上世界坐标（粗接近/PBVS 用）和归一化方位 bearing（末端 IBVS 用），再填成
Detection 发布。相机内参/挂载/姿态换算只维护这一份（见
cuadc_perception/README.md 的检测器库约定）。

契约（Detection 的 position/bearing 语义）唯一权威版本见
docs/spec/05-perception-decision-interface.md。
"""

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np
import yaml


@dataclass
class CameraParams:
    fx: float
    fy: float
    cx: float
    cy: float
    mount_roll_rad: float = 0.0
    mount_pitch_rad: float = 1.5708  # nadir 惯例，见 gazebo_bringup.launch.py 同名参数
    mount_yaw_rad: float = 0.0


def load_camera_params(profile: str) -> CameraParams:
    """从 camera.<profile>.yaml 读相机内参 + 挂载角。内参对应各自 CV 处理分辨率
    （sim 640×480 / real 960×540），换相机 / 重标定只改 yaml。"""
    path = Path(__file__).parent / f'camera.{profile}.yaml'
    with open(path, encoding='utf-8') as fh:
        data = yaml.safe_load(fh) or {}
    return CameraParams(**data)


@dataclass
class VehiclePose:
    position_ned: Tuple[float, float, float]  # (north, east, down)
    orientation_xyzw: Tuple[float, float, float, float]  # 机体->世界四元数


_MIN_RAY_DOWN_COMPONENT = 0.15  # 射线太接近水平（姿态倾斜太大）时反投影解不出合理交点，直接丢弃这一条


def cam_to_world_matrix(camera: CameraParams, pose: VehiclePose):
    """相机系 -> 世界系旋转矩阵。每帧算一次、传给逐个检测复用，避免每个
    轮廓都重算。"""
    body_to_world = _quaternion_to_matrix(*pose.orientation_xyzw)
    cam_to_body = _mount_rotation_matrix(camera.mount_roll_rad, camera.mount_pitch_rad, camera.mount_yaw_rad)
    return body_to_world @ cam_to_body


def normalize_bearing(u, v, camera: CameraParams) -> Tuple[float, float]:
    """像素 -> 归一化像平面方位 (u-cx)/fx, (v-cy)/fy（无量纲，内参已除掉）。
    末端 IBVS 用：决策拿它 + 高度/姿态就能算出米级误差，不需要知道相机内参。
    这是原始像平面方位，不含挂载/姿态旋转——挂载 yaw 非零时的像轴->机体轴
    旋转由决策侧处理（当前默认 mount_yaw=0，像轴≈机体轴）。"""
    return ((u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy)


def reproject_to_cylinder_top(u, v, camera: CameraParams, pose: VehiclePose,
                               cam_to_world, cylinder_height_m):
    """像素 -> 世界坐标：像素反投影成相机系射线，转到世界系，跟筒顶所在平面
    （NED down = -cylinder_height_m，不是地面 down=0）求交——检测到的色块主要
    反映筒的顶面，跟地面求交会引入正比于视线倾角的系统性偏差。返回
    (world_xy, 斜距)，射线太接近水平（姿态倾斜太大）时返回 (None, None)。"""
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
    `_rpy_from_quaternion` 提取 roll/pitch/yaw 用的是同一个约定
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
