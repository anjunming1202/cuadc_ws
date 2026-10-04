"""锥面约束连续降高 IBVS:粗接近世界坐标 PBVS -> 末端在"锥面误差域"内边降边对准
——区别于 stepped_descent_ibvs 的"收敛才降、超差停降"阶梯节奏。

锥面 = min(可见性锥, 精度漏斗):
  可见性锥 = tan(半FOV−margin)·depth(h) − |lever|  ——保证筒全程留在窄轴画幅内;
  精度漏斗 = drop_tol + funnel_slope·(h−drop_alt) ——底部收敛到投放容差。
竖直速度随锥余量比例调度(贴边自动减速、居中满速),并受高度 P 封顶平滑进入 drop_alt。
瞄点随高度迁移:高处对画面中心(可见性最好),降到底全量杆臂——高处精度本不重要。

失效显式分档(mode): track(正常) / coast(短暂丢,保持不降) / climb(持续丢,原地爬升
重捕获——重见自动回 track,爬到 reacquire_ceiling 仍没见退回 coarse 全漏斗) /
endgame(已收敛末段丢,有界盲收口,同 stepped 的收口锁)。全程闭环,盲段仅 endgame 一处
且有界(见用户红线:尽量避免开环)。

组织同 stepped_descent_ibvs:纯 Python 可脱离节点单测,配置在同名 sim/real yaml。
"""

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple

import yaml

from cuadc_decision.approaches.base import (
    ApproachOutput, bearing_to_ground_offset, clip as _clip, nearest_within as _nearest_within)
from cuadc_decision.drop_strategy import Candidate

NAME = 'cone_descent_ibvs'

_OFF_WINDOW = 3  # 冻结偏移取近几帧实测均值:单帧噪声不毒化 coast/收口判据
_SERVO_LP = 2    # 伺服误差取近 2 帧均值:压检测噪声,等效滞后 ~0.05s 远小于回路死区


@dataclass
class ApproachConfig:
    """字段名 = yaml 键。默认值只是兜底(纯单测/import 用),正常从 yaml 读。"""
    cruise_alt_m: float = 2.0
    drop_alt_m: float = 1.0               # 停降高度,h(EKF 解锁点参照)计
    cylinder_height_m: float = 0.3
    fc_rest_clearance_m: float = 0.0      # 飞控静止离地高(ADR-0010 簇①: 0.31)
    camera_below_fc_m: float = 0.0        # 相机在飞控下方(ADR-0010 簇①: 0.20)
    dt_s: float = 0.1
    min_conf: float = 0.5                 # 粗段/锁定置信度门
    min_conf_terminal: float = 0.5        # 末端放低(近距残盘置信度塌落),关联门挡垃圾
    coarse_radius_m: float = 1.0
    refine_gate_m: float = 0.4            # 硬锁世界坐标关联门
    handoff_off_m: float = 0.4
    handoff_speed_mps: float = 0.25
    kp: float = 1.2
    ki: float = 0.4
    kd: float = 0.3
    v_max_mps: float = 1.0
    brake_decel_mps2: float = 0.0         # 抛物线刹车 √(2·a·herr);0=关
    i_zone_m: float = 0.4
    lever_arms_m: Tuple[Tuple[float, float], ...] = ((0.0, 0.0),)  # 按投放次序,同 stepped
    aim_trims_m: Tuple[Tuple[float, float], ...] = ((0.0, 0.0),)   # 经验修正,同 stepped
    max_coast_ticks: int = 4
    endgame_alt_band_m: float = 0.0       # 收口锁带宽(同 stepped);0=关
    # ---- 锥面 ----
    fov_half_deg: float = 17.25           # 纵向(窄轴)半FOV,实测标定 33.4°/2
    fov_margin_deg: float = 4.0           # 可见性余量(俯仰瞬态/圆盘半径的角预算)
    funnel_slope: float = 0.35            # 精度漏斗展开斜率(米误差/米高度)
    # ---- 瞄点迁移 ----
    aim_migrate_top_m: float = 1.8        # 此高度以上瞄画面中心(可见性最大)
    aim_migrate_bot_m: float = 1.15       # 此高度以下全量杆臂瞄点
    # ---- 降高/失效 ----
    descend_rate_mps: float = 0.4
    alt_hold_p: float = 0.0               # 高度P:比例进入 drop_alt+过低爬回;0=关
    climb_rate_mps: float = 0.3           # climb 模式爬升速率(原地重捕获)
    reacquire_ceiling_m: float = 1.9      # 爬到此高度仍未重见 -> 退回 coarse 全漏斗
    # ---- 投放门 ----
    drop_tol_m: float = 0.04
    speed_tol_mps: float = 0.15
    level_tol_rad: float = 0.1
    stable_hold_s: float = 1.0


def load_config(profile: str) -> ApproachConfig:
    path = Path(__file__).parent / f'{NAME}.{profile}.yaml'
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    return ApproachConfig(**data)


def create(config: Optional[ApproachConfig] = None) -> 'ConeDescentIbvs':
    return ConeDescentIbvs(config)


@dataclass
class ApproachState:
    phase: str = 'coarse'                 # coarse -> terminal(内部 mode 细分)
    mode: str = ''                        # 末端子状态: track/coast/climb/endgame(日志可见)
    locked: Optional[Candidate] = None
    lever: Tuple[float, float] = (0.0, 0.0)  # 本次生效瞄准(杆臂+经验修正),enter 时选定
    miss: int = 0
    integ: Tuple[float, float] = (0.0, 0.0)
    prev_off: Optional[Tuple[float, float]] = None
    prev_off_meas: Optional[Tuple[float, float]] = None
    off_window: list = field(default_factory=list)
    last_bearing: Optional[Tuple[float, float]] = None
    stable_since: Optional[float] = None


class ConeDescentIbvs:
    """锥面连续降高。关联用世界(硬锁,姿态无关、区分邻筒)、控制用 bearing(定位无关),
    同 ADR-0005 的两段式;差异全部在末端的降高调度与失效分档。"""

    name = NAME

    def __init__(self, cfg: Optional[ApproachConfig] = None):
        self.cfg = cfg or ApproachConfig()
        self.state = ApproachState()

    def enter(self, ctx):
        i = ctx.drop_task.drop_count
        arms, trims = self.cfg.lever_arms_m, self.cfg.aim_trims_m
        arm = arms[min(i, len(arms) - 1)]
        trim = trims[min(i, len(trims) - 1)]
        self.state = ApproachState(locked=ctx.drop_task.selected_target,
                                   lever=(arm[0] + trim[0], arm[1] + trim[1]))

    def loop(self, ctx, now: Optional[float] = None) -> ApproachOutput:
        now = now if now is not None else time.monotonic()
        h = -ctx.local_position_z if ctx.local_position_z is not None else self.cfg.cruise_alt_m
        conf_gate = self.cfg.min_conf if self.state.phase == 'coarse' else self.cfg.min_conf_terminal
        dets = [d for d in ctx.detections
                if getattr(d, 'target_type', 'cylinder') == 'cylinder' and d.confidence >= conf_gate]
        if self.state.phase == 'coarse':
            return self._coarse(ctx, dets, h)
        return self._terminal(ctx, dets, h, now)

    # ---- 几何 ----

    def _depth(self, h):
        cam_agl = h + self.cfg.fc_rest_clearance_m - self.cfg.camera_below_fc_m
        return cam_agl - self.cfg.cylinder_height_m

    def _cone_radius(self, h):
        """允许水平误差 = min(可见性锥, 精度漏斗),下限 drop_tol。"""
        cfg = self.cfg
        vis = (math.tan(math.radians(cfg.fov_half_deg - cfg.fov_margin_deg)) * self._depth(h)
               - math.hypot(*self.state.lever))
        acc = cfg.drop_tol_m + cfg.funnel_slope * max(0.0, h - cfg.drop_alt_m)
        return max(cfg.drop_tol_m, min(vis, acc))

    def _aim(self, h):
        """瞄点随高度从画面中心迁移到全量杆臂。"""
        cfg = self.cfg
        span = cfg.aim_migrate_top_m - cfg.aim_migrate_bot_m
        w = 1.0 if span <= 0 else max(0.0, min(1.0, (cfg.aim_migrate_top_m - h) / span))
        return (self.state.lever[0] * w, self.state.lever[1] * w)

    def _bearing_to_off(self, bearing, ctx, h):
        roll = (ctx.det_roll_rad if ctx.det_roll_rad is not None else ctx.roll_rad) or 0.0
        pitch = (ctx.det_pitch_rad if ctx.det_pitch_rad is not None else ctx.pitch_rad) or 0.0
        if ctx.det_height_m is not None:
            h = ctx.det_height_m
        return bearing_to_ground_offset(bearing, roll, pitch, self._depth(h))

    # ---- 粗接近(同 stepped) ----

    def _coarse(self, ctx, dets, h) -> ApproachOutput:
        cfg, st = self.cfg, self.state
        target = st.locked
        if target is None:
            return ApproachOutput()
        best = _nearest_within(dets, target.x, target.y, cfg.refine_gate_m)
        if best is not None:
            st.locked = Candidate(best.position.x, best.position.y, best.apparent_size)
            target = st.locked
        aim = (target.x, target.y, -cfg.cruise_alt_m)
        arrived = (ctx.local_position_xy is not None and
                   math.hypot(ctx.local_position_xy[0] - target.x,
                              ctx.local_position_xy[1] - target.y) <= cfg.coarse_radius_m)
        if arrived and best is not None:
            off = self._bearing_to_off((best.bearing_x, best.bearing_y), ctx, h)
            a = self._aim(h)
            e = (off[0] - a[0], off[1] - a[1])
            settled = ctx.horizontal_speed is not None and ctx.horizontal_speed <= cfg.handoff_speed_mps
            if math.hypot(*e) <= cfg.handoff_off_m and settled:
                st.phase, st.mode = 'terminal', 'track'
                st.miss, st.prev_off_meas, st.off_window = 0, off, [off]
                st.integ, st.prev_off, st.stable_since = (0.0, 0.0), None, None
        return ApproachOutput(position_target=aim)

    # ---- 末端(锥面连续降高) ----

    def _terminal(self, ctx, dets, h, now) -> ApproachOutput:
        cfg, st = self.cfg, self.state
        target = st.locked
        best = _nearest_within(dets, target.x, target.y, cfg.refine_gate_m) if target is not None else None
        endgame = False
        if best is not None:
            st.mode = 'track'
            st.locked = Candidate(best.position.x, best.position.y, best.apparent_size)
            st.miss = 0
            st.last_bearing = (best.bearing_x, best.bearing_y)
            off = self._bearing_to_off(st.last_bearing, ctx, h)
            st.off_window = (st.off_window + [off])[-_OFF_WINDOW:]
            st.prev_off_meas = (sum(o[0] for o in st.off_window) / len(st.off_window),
                                sum(o[1] for o in st.off_window) / len(st.off_window))
            lp = st.off_window[-_SERVO_LP:]
            off = (sum(o[0] for o in lp) / len(lp), sum(o[1] for o in lp) / len(lp))
        else:
            st.miss += 1
            endgame = (st.prev_off_meas is not None
                       and math.hypot(st.prev_off_meas[0] - st.lever[0],
                                      st.prev_off_meas[1] - st.lever[1]) < cfg.drop_tol_m
                       and h <= cfg.drop_alt_m + cfg.endgame_alt_band_m)
            if st.prev_off_meas is None or (st.miss > cfg.max_coast_ticks and not endgame):
                # 持续丢失:原地爬升重捕获(重见下 tick 自动回 track);爬到顶仍没见
                # 退回 coarse 全漏斗。清 PID,爬升段不横移。
                st.integ, st.prev_off, st.stable_since = (0.0, 0.0), None, None
                if h >= cfg.reacquire_ceiling_m:
                    st.phase, st.mode, st.miss = 'coarse', '', 0
                    st.prev_off_meas, st.off_window = None, []
                    return self._coarse(ctx, dets, h)
                st.mode = 'climb'
                return ApproachOutput(velocity_target=(0.0, 0.0, -cfg.climb_rate_mps))
            st.mode = 'endgame' if endgame else 'coast'
            off = st.prev_off_meas

        a = self._aim(h)
        e = (off[0] - a[0], off[1] - a[1])
        doff = (0.0, 0.0) if st.prev_off is None else (
            (off[0] - st.prev_off[0]) / cfg.dt_s, (off[1] - st.prev_off[1]) / cfg.dt_s)
        st.prev_off = off
        # 位置误差->速度指令,+kd*doff 是真阻尼(同 stepped,见其注释)。
        raw = (cfg.kp * e[0] + cfg.ki * st.integ[0] + cfg.kd * doff[0],
               cfg.kp * e[1] + cfg.ki * st.integ[1] + cfg.kd * doff[1])
        herr = math.hypot(*e)
        v_lim = cfg.v_max_mps
        if cfg.brake_decel_mps2 > 0.0:
            v_lim = min(v_lim, math.sqrt(2.0 * cfg.brake_decel_mps2 * herr))
        mag = math.hypot(*raw)
        scale = min(1.0, v_lim / mag) if mag > 1e-9 else 0.0
        v = (raw[0] * scale, raw[1] * scale)
        integ = list(st.integ)
        for i in (0, 1):
            if abs(raw[i]) < v_lim and abs(e[i]) < cfg.i_zone_m:
                integ[i] += e[i] * cfg.dt_s
        st.integ = (integ[0], integ[1])

        if h > cfg.drop_alt_m + 1e-3:
            # 锥余量比例调度:贴边减速、出锥停降(coast 也停降);endgame 按高度P盲收口。
            vz = 0.0
            if st.mode in ('track', 'endgame'):
                cone = self._cone_radius(h)
                ratio = max(0.0, min(1.0, (cone - herr) / cone)) if st.mode == 'track' else 1.0
                vz = cfg.descend_rate_mps * ratio
                if cfg.alt_hold_p > 0.0:
                    vz = min(vz, cfg.alt_hold_p * (h - cfg.drop_alt_m))
            return ApproachOutput(velocity_target=(v[0], v[1], vz))

        # 投放高度:高度P保持(过低爬回),查投放门。门用全量杆臂误差(此处 aim==lever)。
        vz_hold = 0.0
        if cfg.alt_hold_p > 0.0:
            vz_hold = _clip(cfg.alt_hold_p * (h - cfg.drop_alt_m), cfg.descend_rate_mps)
        stable = (herr < cfg.drop_tol_m
                  and ctx.horizontal_speed is not None and ctx.horizontal_speed < cfg.speed_tol_mps
                  and ctx.vertical_speed is not None and abs(ctx.vertical_speed) < cfg.speed_tol_mps
                  and ctx.roll_rad is not None and abs(ctx.roll_rad) < cfg.level_tol_rad
                  and ctx.pitch_rad is not None and abs(ctx.pitch_rad) < cfg.level_tol_rad)
        if stable:
            if st.stable_since is None:
                st.stable_since = now
            elif now - st.stable_since >= cfg.stable_hold_s:
                return ApproachOutput(velocity_target=(v[0], v[1], vz_hold), drop_ready=True)
        else:
            st.stable_since = None
        return ApproachOutput(velocity_target=(v[0], v[1], vz_hold))
