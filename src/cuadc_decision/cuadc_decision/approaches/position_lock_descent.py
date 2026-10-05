"""末端锁定目标位置（短窗 PBVS）：粗段/交接/失效处理沿用 cone_descent_ibvs，末端改为
估计筒的世界位置、发位置指令，由飞控位置环对准（ADR-0012）。

末端不再是"像素偏差 × kp → 速度"的外环：外环里串着飞控 ~0.7s 的速度响应会形成极限环
（2026-10-05 实飞）。目标估计 = 最近 est_window_s 关联检测世界坐标（感知按拍照时刻位姿
反投影到筒顶）的中位数；GPS/EKF 慢漂移同时作用在飞机和估计上，窗口内相对误差 cm 级。

降高 = 指令高度斜坡，速率乘锥面余量比例（锥面/瞄点迁移同 cone）；投放门 = 投放口世界位置
到目标估计 < 容差。丢失：短丢保持，持续丢失在估计点上方爬升，已收敛在投放高度丢失照查投放门。

配置：cone 的 sim/real yaml（粗段/失效参数单一来源）+ 本模块同名 yaml 覆盖末端参数。
"""

import math
import statistics
import time
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Optional, Tuple

import yaml

import cuadc_physical
from cuadc_decision.approaches import cone_descent_ibvs as cone
from cuadc_decision.approaches.base import ApproachOutput, physical_fields, nearest_within as _nearest_within
from cuadc_decision.drop_strategy import Candidate

NAME = 'position_lock_descent'


@dataclass
class ApproachConfig(cone.ApproachConfig):
    est_window_s: float = 1.0                         # 目标估计取最近这么久的关联检测
    h_lead_max_m: float = 0.3                         # 指令高度最多领先实际高度这么多（飞控跟不上时不越拉越远）
    camera_offset_m: Tuple[float, float] = (0.0, 0.0)  # 相机相对飞控(前,右)，load_config 从 cuadc_physical 注入


def _read(name):
    with open(Path(__file__).parent / name) as fh:
        return yaml.safe_load(fh) or {}


def load_config(profile: str) -> ApproachConfig:
    data = _read(f'{cone.NAME}.{profile}.yaml')
    data.update(_read(f'{NAME}.{profile}.yaml'))
    data.update(physical_fields())
    data['camera_offset_m'] = tuple(cuadc_physical.load_airframe().camera_position_m[:2])
    return ApproachConfig(**data)


def create(config: Optional[ApproachConfig] = None) -> 'PositionLockDescent':
    return PositionLockDescent(config)


@dataclass
class LockState(cone.ApproachState):
    samples: list = field(default_factory=list)   # (t, x, y) 关联检测的世界坐标
    est: Optional[Tuple[float, float]] = None     # 目标位置估计（窗口中位数；丢失时保持最后值）
    h_cmd: Optional[float] = None                 # 指令高度
    prev_t: Optional[float] = None


class PositionLockDescent(cone.ConeDescentIbvs):
    name = NAME

    def __init__(self, cfg: Optional[ApproachConfig] = None):
        super().__init__(cfg or ApproachConfig())
        self.state = LockState()

    def enter(self, ctx):
        super().enter(ctx)
        self.state = LockState(**{f.name: getattr(self.state, f.name) for f in fields(cone.ApproachState)})

    def _fc_point(self, ctx, offset):
        """飞控应在的位置：目标估计 − R(yaw)·offset（offset 为相对飞控的机体前/右）。"""
        yaw = ctx.yaw_rad or 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        ex, ey = self.state.est
        return (ex - (offset[0] * c - offset[1] * s), ey - (offset[0] * s + offset[1] * c))

    def _to_coarse(self, ctx, dets, h, now) -> ApproachOutput:
        st = self.state
        st.phase, st.mode, st.miss = 'coarse', '', 0
        st.prev_off_meas, st.off_window = None, []
        st.hover_since, st.drop_alt_since, st.relaxed = None, None, False
        st.samples, st.h_cmd, st.stable_since = [], None, None
        if st.est is not None:
            st.locked = Candidate(st.est[0], st.est[1], st.locked.size if st.locked else 0.0)
        st.est, st.prev_t = None, None
        st.reacquires += 1
        if st.reacquires > self.cfg.max_reacquire:
            return ApproachOutput(give_up=('TARGET_LOST_REPEATEDLY', f'lost in terminal {st.reacquires}x '
                                                                     f'(max_reacquire={self.cfg.max_reacquire})'))
        return self._coarse(ctx, dets, h, now)

    def _terminal(self, ctx, dets, h, now) -> ApproachOutput:
        cfg, st = self.cfg, self.state
        give_up = self._terminal_budget(h, now)
        if give_up is not None:
            return ApproachOutput(give_up=give_up)
        dt = cfg.dt_s if st.prev_t is None else min(max(now - st.prev_t, 0.0), 3 * cfg.dt_s)
        st.prev_t = now
        if st.h_cmd is None:  # 刚交接（或从粗段回来）：估计从粗段最后的锁定位置起步
            st.h_cmd, st.samples, st.est = h, [], (st.locked.x, st.locked.y)
        if ctx.local_position_xy is None:
            return ApproachOutput()

        best = _nearest_within(dets, st.est[0], st.est[1], cfg.refine_gate_m)
        if best is not None:
            st.mode, st.miss = 'track', 0
            st.locked = Candidate(best.position.x, best.position.y, best.apparent_size)
            st.samples.append((now, best.position.x, best.position.y))
            st.last_bearing = (best.bearing_x, best.bearing_y)
            off = self._bearing_to_off(st.last_bearing, ctx, h)  # 仅诊断/DROP_AIM 快照
            st.off_window = (st.off_window + [off])[-cone._OFF_WINDOW:]
            st.prev_off_meas = (sum(o[0] for o in st.off_window) / len(st.off_window),
                                sum(o[1] for o in st.off_window) / len(st.off_window))
        else:
            st.miss += 1
        st.samples = [s for s in st.samples if now - s[0] <= cfg.est_window_s]
        if st.samples:
            st.est = (statistics.median(s[1] for s in st.samples), statistics.median(s[2] for s in st.samples))

        lever = st.lever
        port = (cfg.camera_offset_m[0] + lever[0], cfg.camera_offset_m[1] + lever[1])
        px, py = self._fc_point(ctx, port)
        port_err = math.hypot(ctx.local_position_xy[0] - px, ctx.local_position_xy[1] - py)
        at_drop_alt = h <= cfg.drop_alt_m + cfg.drop_alt_band_m
        tol = cfg.relaxed_drop_tol_m if st.relaxed else cfg.drop_tol_m
        if best is None and st.miss > cfg.max_coast_ticks:
            if not (at_drop_alt and port_err < tol):  # 已收敛在投放高度丢失 = endgame，照查投放门
                st.mode, st.stable_since = 'climb', None
                if h >= cfg.reacquire_ceiling_m:
                    return self._to_coarse(ctx, dets, h, now)
                st.h_cmd = min(max(st.h_cmd, h) + cfg.climb_rate_mps * dt, h + cfg.h_lead_max_m)
            else:
                st.mode = 'endgame'
        elif best is None:
            st.mode = 'coast'

        aim = self._aim(h)
        fx, fy = self._fc_point(ctx, (cfg.camera_offset_m[0] + aim[0], cfg.camera_offset_m[1] + aim[1]))
        herr = math.hypot(ctx.local_position_xy[0] - fx, ctx.local_position_xy[1] - fy)
        if st.mode == 'track':
            cone_r = self._cone_radius(h)
            ratio = max(0.0, min(1.0, (cone_r - herr) / cone_r))
            st.h_cmd = max(st.h_cmd - cfg.descend_rate_mps * ratio * dt, h - cfg.h_lead_max_m)
        st.h_cmd = max(st.h_cmd, cfg.drop_alt_m)
        target = (fx, fy, -st.h_cmd)

        stable = (at_drop_alt and st.mode in ('track', 'endgame') and port_err < tol
                  and ctx.horizontal_speed is not None and ctx.horizontal_speed < cfg.speed_tol_mps
                  and ctx.vertical_speed is not None and abs(ctx.vertical_speed) < cfg.speed_tol_mps
                  and ctx.roll_rad is not None and abs(ctx.roll_rad) < cfg.level_tol_rad
                  and ctx.pitch_rad is not None and abs(ctx.pitch_rad) < cfg.level_tol_rad)
        if not stable:
            st.stable_since = None
        elif st.stable_since is None:
            st.stable_since = now
        elif now - st.stable_since >= cfg.stable_hold_s:
            return ApproachOutput(position_target=target, speed_mps=cfg.v_max_mps, drop_ready=True)
        return ApproachOutput(position_target=target, speed_mps=cfg.v_max_mps)  # 末端限速：估计跳变时不在低空快速横移
