"""共享几何助手：像素 <-> 世界坐标 / 归一化方位。检测器（detectors/ 里的）
保持纯图像空间、不碰这里；感知节点在检测器出了像素结果之后，用这里的函数
补上世界坐标（粗接近/PBVS 用）和归一化方位 bearing（末端 IBVS 用），再填成
Detection 发布。相机内参/挂载/姿态换算只维护这一份（见
cuadc_perception/README.md 的检测器库约定）。

本模块的“世界”一律指**起飞坐标系**（原点 = 解锁时飞控位置，x = 起飞时机头方向、y = 右、z = 下，
与东南西北无关），定义见 docs/spec/coordinate-frames.md。契约（Detection 的 position/bearing 语义）
唯一权威版本见 docs/spec/05-perception-decision-interface.md。
"""

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np
import yaml
import cuadc_physical


@dataclass
class CameraParams:
    fx: float
    fy: float
    cx: float
    cy: float
    mount_roll_rad: float = 0.0
    mount_pitch_rad: float = 1.5708  # nadir 惯例，见 gazebo_bringup.launch.py 同名参数
    mount_yaw_rad: float = 0.0
    # 相机光心相对机体原点(FC/EKF)的平移，机体 FRD(前/右/下)，米。真机相机装在
    # FC 前下方，反投影射线须从相机光心而非机体原点出发（见 ADR-0010 簇①）。
    t_forward_m: float = 0.0
    t_right_m: float = 0.0
    t_down_m: float = 0.0
    # 飞控停在地面时离地高。位姿原点 = 解锁时的飞控位置（接口层换算，ADR-0011），地面在其下方这么多。
    fc_rest_clearance_m: float = 0.0


def load_camera_params(profile: str) -> CameraParams:
    """内参从 camera.<profile>.yaml 读（各自 CV 处理分辨率：sim/real 不同，换相机/重标定只改它）；
    相机安装位置/角度与飞控离地高是机体固定几何，从 cuadc_physical 的单一真源读。"""
    path = Path(__file__).parent / f'camera.{profile}.yaml'
    with open(path, encoding='utf-8') as fh:
        data = yaml.safe_load(fh) or {}
    af = cuadc_physical.load_airframe()
    (data['t_forward_m'], data['t_right_m'], data['t_down_m']) = af.camera_position_m
    (data['mount_roll_rad'], data['mount_pitch_rad'], data['mount_yaw_rad']) = af.camera_mount_rad
    data['fc_rest_clearance_m'] = af.fc_rest_clearance_m
    return CameraParams(**data)


@dataclass
class VehiclePose:
    position: Tuple[float, float, float]  # 起飞坐标系 (前, 右, 下)，见 docs/spec/coordinate-frames.md
    orientation_xyzw: Tuple[float, float, float, float]  # 机体->世界四元数


_MIN_RAY_DOWN_COMPONENT = 0.15  # 射线太接近水平（姿态倾斜太大）时反投影解不出合理交点，直接丢弃这一条


def cam_to_world_matrix(camera: CameraParams, pose: VehiclePose):
    """相机系 -> 世界系旋转矩阵。每帧算一次、传给逐个检测复用，避免每个
    轮廓都重算。"""
    body_to_world = _quaternion_to_matrix(*pose.orientation_xyzw)
    cam_to_body = _mount_rotation_matrix(camera.mount_roll_rad, camera.mount_pitch_rad, camera.mount_yaw_rad)
    return body_to_world @ cam_to_body


def camera_origin(camera: CameraParams, pose: VehiclePose):
    """相机光心在起飞坐标系的位置 = 飞控位置(pose.position)
    + 机体系相机平移转到世界。反投影射线从这里出发，抵消相机相对 FC 的安装偏置
    带来的系统性定位偏差（见 ADR-0010 簇①）。每帧算一次、传给逐个检测复用。"""
    body_to_world = _quaternion_to_matrix(*pose.orientation_xyzw)
    t_body = np.array([camera.t_forward_m, camera.t_right_m, camera.t_down_m])
    return np.array(pose.position, dtype=float) + body_to_world @ t_body


def normalize_bearing(u, v, camera: CameraParams) -> Tuple[float, float]:
    """像素 -> 归一化像平面方位 (u-cx)/fx, (v-cy)/fy（无量纲，内参已除掉）。
    末端 IBVS 用：决策拿它 + 高度/姿态就能算出米级误差，不需要知道相机内参。
    这是原始像平面方位，不含挂载/姿态旋转——挂载 yaw 非零时的像轴->机体轴
    旋转由决策侧处理（当前默认 mount_yaw=0，像轴≈机体轴）。"""
    return ((u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy)


def reproject_to_cylinder_top(u, v, camera: CameraParams, cam_origin,
                               cam_to_world, cylinder_height_m):
    """像素 -> 世界坐标：像素反投影成相机系射线，转到世界系，跟筒顶所在平面
    （地面 down = fc_rest_clearance_m，筒顶在其上 cylinder_height_m）求交——检测到的色块主要
    反映筒的顶面，跟地面求交会引入正比于视线倾角的系统性偏差。`cam_origin` 是
    相机光心在起飞坐标系的位置（见 camera_origin）。返回 (world_xy, 斜距)，射线太
    接近水平（姿态倾斜太大）时返回 (None, None)。"""
    d_cam = np.array([(u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy, 1.0])
    d_world = cam_to_world @ d_cam

    origin_down = cam_origin[2]
    if d_world[2] < _MIN_RAY_DOWN_COMPONENT:
        return None, None
    t = (camera.fc_rest_clearance_m - cylinder_height_m - origin_down) / d_world[2]
    if t <= 0:
        return None, None

    world_x = cam_origin[0] + t * d_world[0]
    world_y = cam_origin[1] + t * d_world[1]
    range_m = t * float(np.linalg.norm(d_cam))
    return (world_x, world_y), range_m


def transverse_radius_m(contour, u, v, camera: CameraParams, cam_to_world, range_m):
    """筒的物理半径：轮廓在"横向"（垂直于 画面正下方点 -> 筒 连线）上的宽度。

    竖直的筒侧壁在画面里全部指向正下方点（竖直方向的灭点），只把轮廓沿径向拉长；横向宽度
    恒等于筒口直径，不受侧壁露出多少影响（面积等效半径离正下方 2m 会偏大 30-50%）。
    宽度像素 -> 米：横向位移垂直于视线，ds = dpx * 斜距 * cos(视线离光轴角) / f。"""
    pts = np.asarray(contour, dtype=float).reshape(-1, 2)
    if len(pts) < 3 or range_m is None:
        return None
    down_cam = cam_to_world.T @ np.array([0.0, 0.0, 1.0])  # 世界竖直向下在相机系
    radial = None
    if down_cam[2] > 1e-6:
        nadir = (camera.cx + camera.fx * down_cam[0] / down_cam[2],
                 camera.cy + camera.fy * down_cam[1] / down_cam[2])
        r = np.array([u - nadir[0], v - nadir[1]])
        if np.hypot(*r) > 1.0:
            radial = r / np.hypot(*r)
    if radial is None:  # 几乎正下方：没有侧壁，取两轴宽度均值
        width_px = 0.5 * (np.ptp(pts[:, 0]) + np.ptp(pts[:, 1]))
    else:
        width_px = np.ptp(pts @ np.array([-radial[1], radial[0]]))
    d = np.array([(u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy, 1.0])
    cos_a = 1.0 / float(np.linalg.norm(d))
    f = 0.5 * (camera.fx + camera.fy)
    return 0.5 * float(width_px) * range_m * cos_a / f


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
