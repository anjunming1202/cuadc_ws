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
    # 图像投放门（track 时）：用拍照时刻的图像偏差判断投放口相对筒心（不经 GPS/目标估计），并按偏差变化率预测
    # release_delay_s 后瓶子离开时的偏差；当前值和预测值都在容差内才放料
    image_gate: bool = False
    gate_speed_mps: float = 0.0                       # 投放门水平速度上限；0 = 用 speed_tol_mps
    release_delay_s: float = 0.0                      # 过门到瓶子离开的时间（取决于装瓶时钩子离临界点多远）
    pred_window_s: float = 0.5                        # 偏差变化率取最近这么久的帧做直线拟合
    gate_min_conf: float = 0.0                        # 只有置信度不低于此的帧计入图像投放门
    # 末端目标一致性：检测半径 / 交接时锁定的半径须在 [min, max] 内才关联（0 = 不查）。真筒降到低空半径估大
    # 1.15–1.7 倍；筒被吹走后检测器会把垫边水泥/砖块缺口当成筒（2026-10-06），尺寸和置信度都对不上
    size_ratio_min: float = 0.0
    size_ratio_max: float = 0.0
    allow_endgame: bool = True                        # 投放高度丢检测时按最后估计盲投；关掉则丢失就爬升重捕获
    # 投放门 = drop_tol_m ×（目标筒半径/最小筒半径，size_tolerance 时）× gate_schedule 当前倍数。
    # gate_schedule 为空 = 旧的一档放宽（stable_timeout_s 后 relaxed_drop_tol_m）
    size_tolerance: bool = False
    size_ref_radius_m: float = 0.075                  # 最小筒半径，load_config 从 cuadc_physical 注入
    gate_schedule: Tuple[Tuple[float, float], ...] = ()  # (到投放高度后秒数, 门倍数)，按时间升序
    # 尽力投（保分）：到投放高度这么久还没过门，门改 best_effort_tol_m——偏一点也在筒外 0.5 m 的 B 区内；
    # 再 best_effort_window_s 仍不行才放弃（图像门一直不评估 = 目标不可信）。0 = 关，旧的超时放弃
    best_effort_after_s: float = 0.0
    best_effort_tol_m: float = 0.15
    best_effort_window_s: float = 10.0


def _read(name):
    with open(Path(__file__).parent / name) as fh:
        return yaml.safe_load(fh) or {}


def load_config(profile: str) -> ApproachConfig:
    data = _read(f'{cone.NAME}.{profile}.yaml')
    data.update(_read(f'{NAME}.{profile}.yaml'))
    data.update(physical_fields())
    data['camera_offset_m'] = tuple(cuadc_physical.load_airframe().camera_position_m[:2])
    data['size_ref_radius_m'] = cuadc_physical.load_targets().drop_cylinder_radii_m[0]
    return ApproachConfig(**data)


def create(config: Optional[ApproachConfig] = None) -> 'PositionLockDescent':
    return PositionLockDescent(config)


@dataclass
class LockState(cone.ApproachState):
    samples: list = field(default_factory=list)   # (t, x, y) 关联检测的世界坐标
    est: Optional[Tuple[float, float]] = None     # 目标位置估计（窗口中位数；丢失时保持最后值）
    h_cmd: Optional[float] = None                 # 指令高度
    prev_t: Optional[float] = None
    res_hist: list = field(default_factory=list)  # (t, 前, 右) 图像算出的筒心相对投放口
    gate_res: Optional[Tuple[float, float]] = None   # 最近的图像偏差（3 帧均值，诊断日志用）
    gate_pred: Optional[Tuple[float, float]] = None  # 预测的瓶子离开时偏差
    ref_size: Optional[float] = None              # 交接时锁定目标的半径（末端一致性检查的基准）
    target_r: Optional[float] = None              # 目标是哪个投放筒（真实半径，任务 FSM 进接近时定）
    tol_mult: float = 1.0                         # gate_schedule 当前倍数
    best_effort: bool = False                     # 已改尽力投
    phase_best_effort: bool = False               # 任务 FSM：投放阶段快到了
    gate_tol: Optional[float] = None              # 本拍投放门（诊断日志用）


class PositionLockDescent(cone.ConeDescentIbvs):
    name = NAME

    def __init__(self, cfg: Optional[ApproachConfig] = None):
        super().__init__(cfg or ApproachConfig())
        self.state = LockState()

    def enter(self, ctx):
        super().enter(ctx)
        self.state = LockState(**{f.name: getattr(self.state, f.name) for f in fields(cone.ApproachState)})
        self.state.target_r = getattr(ctx.drop_task, 'target_radius_m', None)

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
        st.tol_mult, st.best_effort = 1.0, False
        st.samples, st.h_cmd, st.stable_since = [], None, None
        st.res_hist, st.gate_res, st.gate_pred, st.ref_size = [], None, None, None
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
        st.phase_best_effort = bool(getattr(ctx.drop_task, 'best_effort_now', False))
        give_up = self._terminal_budget(h, now)
        if give_up is not None:
            return ApproachOutput(give_up=give_up)
        dt = cfg.dt_s if st.prev_t is None else min(max(now - st.prev_t, 0.0), 3 * cfg.dt_s)
        st.prev_t = now
        if st.h_cmd is None:  # 刚交接（或从粗段回来）：估计从粗段最后的锁定位置起步
            st.h_cmd, st.samples, st.est = h, [], (st.locked.x, st.locked.y)
            st.ref_size = st.locked.size if st.locked.size > 0 else None
        if ctx.local_position_xy is None:
            return ApproachOutput()

        if cfg.size_ratio_max > 0 and st.ref_size:
            dets = [d for d in dets if cfg.size_ratio_min * st.ref_size <= d.apparent_size <= cfg.size_ratio_max * st.ref_size]
        best = _nearest_within(dets, st.est[0], st.est[1], cfg.refine_gate_m)
        if best is not None:
            st.mode, st.miss = 'track', 0
            st.locked = Candidate(best.position.x, best.position.y, best.apparent_size)
            st.samples.append((now, best.position.x, best.position.y))
            st.last_bearing = (best.bearing_x, best.bearing_y)
            off = self._bearing_to_off(st.last_bearing, ctx, h)  # 图像投放门 + DROP_AIM 快照
            if best.confidence >= cfg.gate_min_conf:
                st.res_hist.append((now, off[0] - st.lever[0], off[1] - st.lever[1]))
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
        tol = st.gate_tol = self._gate_tol()
        if best is None and st.miss > cfg.max_coast_ticks:
            if not (cfg.allow_endgame and at_drop_alt and port_err < tol):  # 已收敛在投放高度丢失 = endgame，照查投放门
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

        st.res_hist = [r for r in st.res_hist if now - r[0] <= cfg.pred_window_s]
        aligned = port_err < tol
        if cfg.image_gate and st.mode == 'track':  # endgame（已丢检测）没有图像，仍按目标估计
            aligned = self._image_aligned(tol)
        stable = (at_drop_alt and st.mode in ('track', 'endgame') and aligned
                  and ctx.horizontal_speed is not None
                  and ctx.horizontal_speed < (cfg.gate_speed_mps or cfg.speed_tol_mps)
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

    def _gate_tol(self):
        cfg, st = self.cfg, self.state
        if st.best_effort:
            return cfg.best_effort_tol_m
        scale = st.target_r / cfg.size_ref_radius_m if cfg.size_tolerance and st.target_r else 1.0
        if cfg.gate_schedule:
            return cfg.drop_tol_m * scale * st.tol_mult
        return cfg.relaxed_drop_tol_m if st.relaxed else cfg.drop_tol_m * scale

    def _terminal_budget(self, h, now):
        """有 gate_schedule / 尽力投时：投放门按时间表逐档放宽；best_effort_after_s 后（或任务 FSM 说投放阶段
        快到了）改尽力投，再 best_effort_window_s 仍没投才放弃。都没配时同 cone 的一档放宽 + 超时放弃。"""
        cfg, st = self.cfg, self.state
        if not cfg.gate_schedule and cfg.best_effort_after_s <= 0:
            return super()._terminal_budget(h, now)
        if st.terminal_since is not None and now - st.terminal_since > cfg.terminal_timeout_s:
            return ('TERMINAL_TIMEOUT', f'{now - st.terminal_since:.0f}s in terminal for this target '
                                        f'(reacquires={st.reacquires}), h={h:.2f}m')
        if st.drop_alt_since is None and h <= cfg.drop_alt_m + cfg.drop_alt_band_m:
            st.drop_alt_since = now
        if st.drop_alt_since is None:
            return None
        at_alt_s = now - st.drop_alt_since
        if cfg.best_effort_after_s > 0:
            if at_alt_s > cfg.best_effort_after_s + cfg.best_effort_window_s:
                return ('DROP_GATE_TIMEOUT', f'{at_alt_s:.0f}s since reaching drop alt, best-effort gate '
                                             f'{cfg.best_effort_tol_m:.2f}m not passed either')
            why = (f'{cfg.best_effort_after_s:.0f}s at drop alt' if at_alt_s > cfg.best_effort_after_s
                   else 'drop phase ending' if st.phase_best_effort else None)
            if why and not st.best_effort:
                st.best_effort = True
                st.pending_event = ('DROP_BEST_EFFORT', f'{why}: gate -> {cfg.best_effort_tol_m:.2f}m')
                return None
        elif at_alt_s > cfg.stable_timeout_s + cfg.relaxed_timeout_s:
            return ('DROP_GATE_TIMEOUT', f'{at_alt_s:.0f}s since reaching drop alt without passing the gate')
        if cfg.gate_schedule and not st.best_effort:
            mult = next((m for t, m in reversed(cfg.gate_schedule) if at_alt_s >= t), 1.0)
            if mult != st.tol_mult:
                st.tol_mult = mult
                st.pending_event = ('DROP_GATE_RELAXED', f'{at_alt_s:.0f}s at drop alt: gate -> {self._gate_tol():.3f}m '
                                                         f'(target r={st.target_r})')
        return None

    def _image_aligned(self, tol):
        """图像偏差（最近 3 帧均值）和按变化率外推 release_delay_s 后的偏差都 < tol。帧不够拟合时不放料。"""
        st, cfg = self.state, self.cfg
        hist = st.res_hist
        st.gate_res = st.gate_pred = None
        if len(hist) < 3 or hist[-1][0] - hist[0][0] < 0.2:
            return False
        last = hist[-3:]
        cur = (sum(r[1] for r in last) / 3, sum(r[2] for r in last) / 3)
        tm = sum(r[0] for r in hist) / len(hist)
        var = sum((r[0] - tm) ** 2 for r in hist)
        rate = tuple(sum((r[0] - tm) * r[k] for r in hist) / var for k in (1, 2))
        pred = (cur[0] + rate[0] * cfg.release_delay_s, cur[1] + rate[1] * cfg.release_delay_s)
        st.gate_res, st.gate_pred = cur, pred
        return math.hypot(*cur) < tol and math.hypot(*pred) < tol
