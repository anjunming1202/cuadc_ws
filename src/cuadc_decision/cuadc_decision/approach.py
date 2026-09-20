"""投放接近的可替换「阶段策略」单元。

任务状态机(SEARCH->APPROACH->DROP,见 docs/spec/06-planning.md)与 transition
契约是固定框架;`APPROACH_DROP_TARGET` 内部**怎么把飞机对到筒上**是可替换的
部分——每个 approach 是一个类,注册进 APPROACHES,改 ACTIVE_APPROACH 切换/实验,
不动状态机主干(用法同 drop_strategy.py,粒度更大)。约定见
cuadc_decision/README.md;末端用 IBVS 的决策见
docs/spec/adr/0005-terminal-align-visual-servoing.md。

纯 Python,不依赖 rclpy——可脱离节点单测(含末端 IBVS 控制律)。approach 只消费
MissionContext(遥测 + 检测),吐 ApproachOutput(位置或速度指令 + 是否可投),不碰
ROS 类型;decision_node 把它翻译成 position_target / velocity_target。
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from cuadc_decision.drop_strategy import Candidate


@dataclass
class ApproachConfig:
    """approach 的可调参数。默认是离线 sim 起点值(见 cv_explore/servo_align),
    上 Gazebo/真机按真实延迟/动力学重调;sim/real 分别用各自 yaml 覆盖
    (见 cuadc_decision/README.md 配置分层)。"""
    cruise_alt_m: float = 2.0
    drop_alt_m: float = 1.0
    cylinder_height_m: float = 0.3        # 圆筒高;末端反算深度=离地高-筒高(相机到筒顶平面),见 _bearing_to_off
    dt_s: float = 0.1                     # tick 周期(对应 decision tick_rate_hz=10),积分/微分用
    min_conf: float = 0.5                 # 只对置信度够的检测伺服
    # 粗接近(PBVS) -> 交接
    coarse_radius_m: float = 1.0          # 水平进到这么近(且看得见)才考虑切末端
    refine_gate_m: float = 0.4            # 硬锁:粗段只用离锁定目标这么近的检测刷新
    handoff_off_m: float = 0.6            # 末端交接:估计水平误差小于此且有检测才切
    # 末端 IBVS PID(条件积分 + D on measurement)
    kp: float = 1.2
    ki: float = 0.4
    kd: float = 0.3
    v_max_mps: float = 1.0
    i_zone_m: float = 0.4                 # 只在近端积分,防大接近段积分饱和回弹
    lever_arm_m: Tuple[float, float] = (0.0, 0.0)  # 机体系(前,右)投放机构杆臂,米;sim 默认见 yaml、real empirical 测
    # bearing 跟踪器(硬锁:门 + 连续失配后仅在门内重锁,绝不跳别的筒)
    track_alpha: float = 0.5
    track_beta: float = 0.1
    track_gate_norm: float = 0.12         # 归一化 bearing 关联门(正常跟踪)
    reacquire_gate_norm: float = 0.25     # 重锁门:比关联门宽,但仍围绕上次 bearing
    max_coast_ticks: int = 4
    # 降高
    descend_rate_mps: float = 0.4
    descend_pause_off_m: float = 0.30     # 水平误差大于此暂停降高、先追回来
    # 投放门(±3-5cm 目标)
    drop_tol_m: float = 0.04
    speed_tol_mps: float = 0.15
    level_tol_rad: float = 0.1            # roll/pitch 摆平
    stable_hold_s: float = 1.0            # 上述连续满足这么久才可投


@dataclass
class ApproachState:
    phase: str = 'coarse'                 # coarse -> terminal
    locked: Optional[Candidate] = None    # 锁定目标(世界坐标),粗段用
    tx: Optional[Tuple[float, float]] = None      # 跟踪的 bearing (bx, by)
    tv: Tuple[float, float] = (0.0, 0.0)
    miss: int = 0
    integ: Tuple[float, float] = (0.0, 0.0)
    prev_off: Optional[Tuple[float, float]] = None
    stable_since: Optional[float] = None


@dataclass
class ApproachOutput:
    position_target: Optional[Tuple[float, float, float]] = None  # (x,y,z) 局部 NED,粗段
    velocity_target: Optional[Tuple[float, float, float]] = None  # (前,右,下) 机体系速度,末端
    drop_ready: bool = False


def _clip(v, lim):
    return max(-lim, min(lim, v))


def _nearest_within(dets, x, y, gate):
    best, best_dist = None, gate
    for d in dets:
        dist = math.hypot(d.position.x - x, d.position.y - y)
        if dist < best_dist:
            best_dist, best = dist, d
    return best


class CoarsePbvsThenTerminalIbvs:
    """粗接近用世界坐标 PBVS 把飞机带到目标附近(把筒弄进视野),末端切像素
    IBVS 精对准——对定位误差免疫(见 ADR-0005)。全程硬锁选定目标:粗段按
    世界坐标门、末段按 bearing 门关联,别的筒进视野不跳。"""

    name = 'coarse_pbvs_then_terminal_ibvs'

    def __init__(self, cfg: Optional[ApproachConfig] = None):
        self.cfg = cfg or ApproachConfig()
        self.state = ApproachState()

    def enter(self, ctx):
        """进入 APPROACH 时调:以 SEARCH 选定的目标为锁定种子,清内部状态。"""
        self.state = ApproachState(locked=ctx.drop_task.selected_target)

    def loop(self, ctx, now: Optional[float] = None) -> ApproachOutput:
        now = now if now is not None else time.monotonic()
        h = -ctx.local_position_z if ctx.local_position_z is not None else self.cfg.cruise_alt_m
        dets = [d for d in ctx.detections
                if getattr(d, 'target_type', 'cylinder') == 'cylinder' and d.confidence >= self.cfg.min_conf]
        if self.state.phase == 'coarse':
            return self._coarse(ctx, dets, h)
        return self._terminal(ctx, dets, h, now)

    # ---- 粗接近(PBVS,世界坐标) ----

    def _coarse(self, ctx, dets, h) -> ApproachOutput:
        cfg, st = self.cfg, self.state
        target = st.locked
        if target is None:
            return ApproachOutput()  # 没锁定目标(异常),不发指令
        best = _nearest_within(dets, target.x, target.y, cfg.refine_gate_m)  # 硬锁:门内最近才认
        if best is not None:
            st.locked = Candidate(best.position.x, best.position.y, best.apparent_size)
            target = st.locked
        aim = (target.x, target.y, -cfg.cruise_alt_m)
        arrived = (ctx.local_position_xy is not None and
                   math.hypot(ctx.local_position_xy[0] - target.x,
                              ctx.local_position_xy[1] - target.y) <= cfg.coarse_radius_m)
        if arrived and best is not None:
            bearing = (best.bearing_x, best.bearing_y)
            off = self._bearing_to_off(bearing, ctx, h)
            e = (off[0] - cfg.lever_arm_m[0], off[1] - cfg.lever_arm_m[1])
            if math.hypot(*e) <= cfg.handoff_off_m:
                st.phase = 'terminal'
                st.tx, st.tv, st.miss = bearing, (0.0, 0.0), 0
                st.integ, st.prev_off, st.stable_since = (0.0, 0.0), None, None
        return ApproachOutput(position_target=aim)

    # ---- 末端对准(IBVS,像素误差 -> 机体系速度) ----

    def _terminal(self, ctx, dets, h, now) -> ApproachOutput:
        cfg, st = self.cfg, self.state
        self._track(dets)
        if st.tx is None:
            return ApproachOutput(velocity_target=(0.0, 0.0, 0.0))  # 彻底没目标:悬停,不乱冲

        off = self._bearing_to_off(st.tx, ctx, h)
        e = (off[0] - cfg.lever_arm_m[0], off[1] - cfg.lever_arm_m[1])
        doff = (0.0, 0.0) if st.prev_off is None else (
            (off[0] - st.prev_off[0]) / cfg.dt_s, (off[1] - st.prev_off[1]) / cfg.dt_s)
        st.prev_off = off
        raw = (cfg.kp * e[0] + cfg.ki * st.integ[0] - cfg.kd * doff[0],
               cfg.kp * e[1] + cfg.ki * st.integ[1] - cfg.kd * doff[1])
        v = (_clip(raw[0], cfg.v_max_mps), _clip(raw[1], cfg.v_max_mps))
        integ = list(st.integ)
        for i in (0, 1):  # 条件积分:近端且未饱和才积分
            if abs(raw[i]) < cfg.v_max_mps and abs(e[i]) < cfg.i_zone_m:
                integ[i] += e[i] * cfg.dt_s
        st.integ = (integ[0], integ[1])

        herr = math.hypot(*e)
        if h > cfg.drop_alt_m + 1e-3:  # 还在降高:够居中才降,飘了先追回来
            vz = cfg.descend_rate_mps if herr < cfg.descend_pause_off_m else 0.0
            return ApproachOutput(velocity_target=(v[0], v[1], vz))

        # 到投放高度:停降,检查投放门(用对地遥测速度,顶风自然满足)
        stable = (herr < cfg.drop_tol_m
                  and ctx.horizontal_speed is not None and ctx.horizontal_speed < cfg.speed_tol_mps
                  and ctx.vertical_speed is not None and abs(ctx.vertical_speed) < cfg.speed_tol_mps
                  and ctx.roll_rad is not None and abs(ctx.roll_rad) < cfg.level_tol_rad
                  and ctx.pitch_rad is not None and abs(ctx.pitch_rad) < cfg.level_tol_rad)
        if stable:
            if st.stable_since is None:
                st.stable_since = now
            elif now - st.stable_since >= cfg.stable_hold_s:
                return ApproachOutput(velocity_target=(v[0], v[1], 0.0), drop_ready=True)
        else:
            st.stable_since = None
        return ApproachOutput(velocity_target=(v[0], v[1], 0.0))

    def _track(self, dets):
        """alpha-beta 跟踪器,bearing 空间硬锁:关联门内正常更新,连续失配后
        只在(较宽的)重锁门内重锁,门外的检测(别的筒)一律不认。"""
        cfg, st = self.cfg, self.state
        meas = self._associate(dets)  # 仅返回重锁门内、离 tx 最近的 bearing,否则 None
        if meas is None:
            st.miss += 1
            st.tx = (st.tx[0] + st.tv[0] * cfg.dt_s, st.tx[1] + st.tv[1] * cfg.dt_s)  # coast
            return
        pred = (st.tx[0] + st.tv[0] * cfg.dt_s, st.tx[1] + st.tv[1] * cfg.dt_s)
        r = (meas[0] - pred[0], meas[1] - pred[1])
        if math.hypot(*r) <= cfg.track_gate_norm:
            st.tx = (pred[0] + cfg.track_alpha * r[0], pred[1] + cfg.track_alpha * r[1])
            st.tv = (st.tv[0] + cfg.track_beta / cfg.dt_s * r[0],
                     st.tv[1] + cfg.track_beta / cfg.dt_s * r[1])
            st.miss = 0
        else:
            st.miss += 1
            if st.miss >= cfg.max_coast_ticks:  # 连续失配:在重锁门内重锁(associate 已保证门内)
                st.tx, st.tv, st.miss = meas, (0.0, 0.0), 0
            else:
                st.tx = pred  # 先 coast 几帧

    def _associate(self, dets):
        """返回离当前 tracked bearing 最近、且在重锁门内的检测 bearing;门外
        (别的位置的筒)返回 None——硬锁的关键,进视野的别的筒不会被吸附。"""
        st, cfg = self.state, self.cfg
        if st.tx is None:
            return None
        best, best_dist = None, cfg.reacquire_gate_norm
        for d in dets:
            dist = math.hypot(d.bearing_x - st.tx[0], d.bearing_y - st.tx[1])
            if dist < best_dist:
                best_dist, best = dist, (d.bearing_x, d.bearing_y)
        return best

    def _bearing_to_off(self, bearing, ctx, h):
        """归一化 bearing -> 目标相对「正下方」的水平偏移(米,机体前/右),含姿态
        补偿。下视固定挂载:相机系射线 (bx,by,1) 在机体系是 (-by,bx,1)(见
        cuadc_perception/geometry.py 挂载约定 pitch=+pi/2);再用 roll/pitch 转到
        水平面 R_y(pitch)R_x(roll) 求与地面交点。yaw 不需要——机体系速度指令本身
        就是相对当前航向的。姿态取自 IMU(准),深度 h 只缩放增益不改终点。"""
        bx, by = bearing
        roll = ctx.roll_rad or 0.0
        pitch = ctx.pitch_rad or 0.0
        r = (-by, bx, 1.0)  # 机体系射线(前,右,下)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        r1 = (r[0], cr * r[1] - sr * r[2], sr * r[1] + cr * r[2])          # R_x(roll) @ r
        lvl = (cp * r1[0] + sp * r1[2], r1[1], -sp * r1[0] + cp * r1[2])   # R_y(pitch) @ r1
        depth = h - self.cfg.cylinder_height_m  # 相机到筒顶平面的竖直距离(不是离地高),否则偏心杆臂会系统性偏
        if lvl[2] <= 1e-6 or depth <= 0.0:
            return (0.0, 0.0)  # 射线接近水平 / 高度异常,解不出合理交点
        t = depth / lvl[2]
        return (t * lvl[0], t * lvl[1])  # (前, 右)


APPROACHES = {
    'coarse_pbvs_then_terminal_ibvs': CoarsePbvsThenTerminalIbvs,
}
ACTIVE_APPROACH = 'coarse_pbvs_then_terminal_ibvs'  # 改这行切换/实验(以后有新形态再加)


def create_active(cfg: Optional[ApproachConfig] = None):
    return APPROACHES[ACTIVE_APPROACH](cfg)
