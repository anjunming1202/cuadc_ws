"""像素 → 世界的反投影 / 射线几何。借自 cuadc_perception/cv_pipeline.py 的
反投影思路,改成吃本数据集的**欧拉角**姿态(index.csv 的 roll/pitch/yaw)。

约定与告警(务必先读):
- 姿态用 ArduPilot/MAVLink 惯例:roll/pitch/yaw 是 body 相对 NED,body→NED
  旋转 = Rz(yaw)·Ry(pitch)·Rx(roll)。
- 相机挂载 `MOUNT_RPY_RAD` 默认 nadir(pitch=+π/2),沿用 cuadc_perception 的
  相机光轴约定(x-右, y-下, z-前)。**但本数据集是手持、rig 外参没填**
  (run_meta 里 camera_to_imu 全 0),这个挂载/光轴约定**没在本数据上验证过**,
  绝对反投影尺度会偏。
- **可靠的原语是 `world_ray()`(射线本身,免高度)**;`intersect_plane()` 需要高度、
  这里高度噪声大 → 只当粗略参考。无标注验证优先用**射线交汇密度**,不要依赖单帧落点。
"""
import math
from typing import Optional, Tuple

import numpy as np

IMG_W, IMG_H = 1920, 1080
F_PX_APPROX = 1450.0            # item3 估计:1920 宽下 ~1450px(HFoV~67°);有真标定就覆盖
CX_DEFAULT, CY_DEFAULT = IMG_W / 2.0, IMG_H / 2.0
MOUNT_RPY_RAD = (0.0, math.pi / 2.0, 0.0)  # 相机→机体挂载,默认 nadir(未验证,见文件头)
_MIN_RAY_DOWN = 0.15           # 射线太接近水平时反投影解不出合理交点,丢弃


def euler_to_R_body_to_ned(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    r, p, y = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _cam_to_body(mount_rpy_rad=MOUNT_RPY_RAD) -> np.ndarray:
    roll, pitch, yaw = mount_rpy_rad
    cp, sp = math.cos(pitch), math.sin(pitch)
    ry = np.array([[cp, 0, -sp], [0, 1, 0], [sp, 0, cp]])
    cr, sr = math.cos(roll), math.sin(roll)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    # 相机光轴(x-右,y-下,z-前)在零挂载角时对齐机体前向,同 cuadc_perception
    base = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    return rz @ ry @ rx @ base


def world_ray(u: float, v: float, roll_deg: float, pitch_deg: float, yaw_deg: float,
              fx: float = F_PX_APPROX, fy: float = F_PX_APPROX,
              cx: float = CX_DEFAULT, cy: float = CY_DEFAULT,
              mount_rpy_rad=MOUNT_RPY_RAD) -> np.ndarray:
    """像素 (u,v) → NED 系单位射线方向(免高度的可靠原语)。"""
    d_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    cam_to_ned = euler_to_R_body_to_ned(roll_deg, pitch_deg, yaw_deg) @ _cam_to_body(mount_rpy_rad)
    d = cam_to_ned @ d_cam
    return d / np.linalg.norm(d)


def intersect_plane(origin_ned: Tuple[float, float, float], dir_ned: np.ndarray,
                    plane_down: float) -> Optional[Tuple[Tuple[float, float], float]]:
    """射线交水平面 down=plane_down(如筒顶 = -筒高;地面 = 0)。
    返回 ((north, east), 斜距) 或 None(射线不够朝下 / 交点在身后)。**需要可信高度,慎用**。
    """
    dz = float(dir_ned[2])
    if dz < _MIN_RAY_DOWN:
        return None
    t = (plane_down - origin_ned[2]) / dz
    if t <= 0:
        return None
    north = origin_ned[0] + t * float(dir_ned[0])
    east = origin_ned[1] + t * float(dir_ned[1])
    return (north, east), t
