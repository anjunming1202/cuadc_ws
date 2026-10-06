"""Approach 返回契约 + 各 approach 共用的纯几何/工具函数。放 base(无依赖)避免和
注册表 __init__ 循环导入——所有 approach 都吐同一种 ApproachOutput,任务 FSM 统一
消费(位置或速度指令 + 是否可投),decision_node 再翻成 position_target /
velocity_target。几何函数收在这防多模块各持一份漂移。"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import cuadc_physical


@dataclass
class ApproachOutput:
    position_target: Optional[Tuple[float, float, float]] = None  # (x,y,z) 起飞坐标系(前/右/下),粗段
    velocity_target: Optional[Tuple[float, float, float]] = None  # (前,右,下) 机体系速度,末端
    drop_ready: bool = False
    give_up: Optional[Tuple[str, str]] = None  # (失效代码, 原因)：本目标对不上，任务 FSM 拉黑它换下一个
    event: Optional[Tuple[str, str]] = None    # (代码, 原因)：值得记一条 FAULT 的处理（改锁、放宽投放门）
    speed_mps: Optional[float] = None          # 位置指令的水平限速；None = 任务按区域默认


def physical_fields():
    """approach 配置里属于固定物理量的字段（机体几何、筒高），统一取自 cuadc_physical（单一真源），
    不在 yaml 里重复。"""
    af = cuadc_physical.load_airframe()
    return {'fc_rest_clearance_m': af.fc_rest_clearance_m,
            'camera_below_fc_m': af.camera_below_fc_m,
            'lever_arms_m': af.lever_arms_from_camera(),
            'cylinder_height_m': cuadc_physical.load_targets().drop_cylinder_height_m}


def clip(v, lim):
    return max(-lim, min(lim, v))


def nearest_within(dets, x, y, gate):
    """世界坐标最近门关联:门内最近的检测,没有则 None(硬锁的共用原语)。"""
    best, best_dist = None, gate
    for d in dets:
        dist = math.hypot(d.position.x - x, d.position.y - y)
        if dist < best_dist:
            best_dist, best = dist, d
    return best


def bearing_to_ground_offset(bearing, roll, pitch, depth):
    """归一化 bearing -> 目标相对「正下方」的水平偏移(米,机体前/右),含姿态补偿。
    下视固定挂载:相机系射线 (bx,by,1) 在机体系是 (-by,bx,1)(见
    cuadc_perception/geometry.py 挂载约定 pitch=+pi/2);再用 roll/pitch 转到水平面
    R_y(pitch)R_x(roll) 求与 depth(相机到目标平面的竖直距离)处平面的交点。yaw 不需要
    ——机体系速度指令本身就是相对当前航向的。"""
    bx, by = bearing
    r = (-by, bx, 1.0)  # 机体系射线(前,右,下)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    r1 = (r[0], cr * r[1] - sr * r[2], sr * r[1] + cr * r[2])          # R_x(roll) @ r
    lvl = (cp * r1[0] + sp * r1[2], r1[1], -sp * r1[0] + cp * r1[2])   # R_y(pitch) @ r1
    if lvl[2] <= 1e-6 or depth <= 0.0:
        return (0.0, 0.0)  # 射线接近水平 / 深度异常,解不出合理交点
    t = depth / lvl[2]
    return (t * lvl[0], t * lvl[1])  # (前, 右)
