"""投放接近的第一份「阶段策略」:粗接近世界坐标 PBVS(把筒弄进视野)-> 末端
像素 IBVS 精对准(对定位误差免疫)。决策见
docs/spec/adr/0005-terminal-align-visual-servoing.md;组织/迁移约定见
cuadc_decision/README.md(一个 approach 一个模块 + 同名 sim/real yaml 跟着放)。

纯 Python,不依赖 rclpy——可脱离节点单测(含末端 IBVS 控制律)。approach 只消费
MissionContext(遥测 + 检测),吐 ApproachOutput,不碰 ROS 类型。可调参数(增益/
门限/降高率/投放容差…)由 `<此目录>/<NAME>.{sim,real}.yaml` 提供,`load_config`
读取,场景启动脚本选 profile。
"""

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import yaml

from cuadc_decision.approaches.base import ApproachOutput
from cuadc_decision.drop_strategy import Candidate

NAME = 'coarse_pbvs_then_terminal_ibvs'


@dataclass
class ApproachConfig:
    """approach 的可调参数。字段名 = yaml 键。默认值只是兜底(纯单测/import 用),
    正常从 <NAME>.<profile>.yaml 读:sim 是 Gazebo 验证值,real 上真机按真实
    延迟/动力学重调。"""
    cruise_alt_m: float = 2.0
    drop_alt_m: float = 1.0
    cylinder_height_m: float = 0.3        # 圆筒高;末端反算深度=离地高-筒高(相机到筒顶平面),见 _bearing_to_off
    dt_s: float = 0.1                     # tick 周期(对应 decision tick_rate_hz=10),积分/微分用
    min_conf: float = 0.5                 # 只对置信度够的检测伺服
    # 粗接近(PBVS) -> 交接
    coarse_radius_m: float = 1.0          # 水平进到这么近(且看得见)才考虑切末端
    refine_gate_m: float = 0.4            # 硬锁世界坐标关联门:粗/末段都只认离锁定目标这么近的检测(筒间距~1.5m,足够区分)
    handoff_off_m: float = 0.6            # 末端交接:估计水平误差小于此且有检测才切
    handoff_speed_mps: float = 0.4        # 交接还要求对地速度降到此以下:粗段先在目标上方停稳再交接,否则带大动量冲进末端必过冲
    # 末端 IBVS PID(条件积分 + D on measurement)
    kp: float = 1.2
    ki: float = 0.4
    kd: float = 0.3
    v_max_mps: float = 1.0
    i_zone_m: float = 0.4                 # 只在近端积分,防大接近段积分饱和回弹
    lever_arm_m: Tuple[float, float] = (0.0, 0.0)  # 机体系(前,右)投放机构杆臂,米;sim 无物理机构=0、real empirical 测
    max_coast_ticks: int = 4              # 末段连续失配超过此帧数就悬停,不靠陈旧偏移乱冲
    # 降高
    descend_rate_mps: float = 0.4
    descend_pause_off_m: float = 0.30     # 水平误差大于此暂停降高、先追回来
    # 投放门(±3-5cm 目标)
    drop_tol_m: float = 0.04
    speed_tol_mps: float = 0.15
    level_tol_rad: float = 0.1            # roll/pitch 摆平
    stable_hold_s: float = 1.0            # 上述连续满足这么久才可投


def load_config(profile: str) -> ApproachConfig:
    """从 `<此目录>/<NAME>.<profile>.yaml` 读配置(profile 通常 'sim'/'real')。"""
    path = Path(__file__).parent / f'{NAME}.{profile}.yaml'
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    return ApproachConfig(**data)


def create(config: Optional[ApproachConfig] = None) -> 'CoarsePbvsThenTerminalIbvs':
    return CoarsePbvsThenTerminalIbvs(config)


@dataclass
class ApproachState:
    phase: str = 'coarse'                 # coarse -> terminal
    locked: Optional[Candidate] = None    # 锁定目标(世界坐标),粗/末段硬锁都用它关联
    miss: int = 0                         # 末段连续失配帧数(丢检测),超阈值悬停
    integ: Tuple[float, float] = (0.0, 0.0)
    prev_off: Optional[Tuple[float, float]] = None       # 上一帧偏移,PID 的 D 项(on measurement)用
    prev_off_meas: Optional[Tuple[float, float]] = None  # 上一次有检测时的偏移,短暂丢失时 coast 用
    stable_since: Optional[float] = None


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
    IBVS 精对准——对定位误差免疫(见 ADR-0005)。全程硬锁选定目标:粗/末段
    都按世界坐标最近门关联(姿态无关、能区分相邻筒),别的筒进视野不跳;末段
    伺服用 bearing 姿态补偿偏移(定位无关)——关联用世界、控制用 bearing。"""

    name = NAME

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
            settled = ctx.horizontal_speed is not None and ctx.horizontal_speed <= cfg.handoff_speed_mps
            if math.hypot(*e) <= cfg.handoff_off_m and settled:
                st.phase = 'terminal'
                st.miss, st.prev_off_meas = 0, off
                st.integ, st.prev_off, st.stable_since = (0.0, 0.0), None, None
        return ApproachOutput(position_target=aim)

    # ---- 末端对准(IBVS,像素误差 -> 机体系速度) ----

    def _terminal(self, ctx, dets, h, now) -> ApproachOutput:
        cfg, st = self.cfg, self.state
        # 硬锁沿用粗段的世界坐标最近门关联:姿态无关——裸 bearing 会随机身俯仰/
        # 横滚摆动(高速交接的减速俯仰能把 bearing 甩出关联门、误判丢锁),世界
        # 坐标不会;且定位偏置在"检测与锁定目标的相对量"里抵消,能稳定区分相邻
        # 筒。伺服仍用 bearing 反算的姿态补偿偏移(定位无关、cm 级,见 ADR-0005)
        # ——关联用世界、控制用 bearing,各取所长。
        target = st.locked
        best = _nearest_within(dets, target.x, target.y, cfg.refine_gate_m) if target is not None else None
        if best is not None:
            st.locked = Candidate(best.position.x, best.position.y, best.apparent_size)
            st.miss = 0
            off = self._bearing_to_off((best.bearing_x, best.bearing_y), ctx, h)
            st.prev_off_meas = off
        else:
            st.miss += 1
            if st.prev_off_meas is None or st.miss > cfg.max_coast_ticks:
                # 彻底丢目标:悬停(零机体速度),绝不靠陈旧偏移继续冲(那正是之前
                # 跑飞的原因),清 PID 状态,等重新关联。
                st.integ, st.prev_off, st.stable_since = (0.0, 0.0), None, None
                return ApproachOutput(velocity_target=(0.0, 0.0, 0.0))
            off = st.prev_off_meas  # 短暂丢失:保持上次偏移 coast 几帧

        e = (off[0] - cfg.lever_arm_m[0], off[1] - cfg.lever_arm_m[1])
        doff = (0.0, 0.0) if st.prev_off is None else (
            (off[0] - st.prev_off[0]) / cfg.dt_s, (off[1] - st.prev_off[1]) / cfg.dt_s)
        st.prev_off = off
        # 本控制器是"位置误差 e=off-lever -> 速度指令",不是常规"误差 -> 力"。
        # 接近目标时 off 减小、doff<0,阻尼项应 +kd*doff(使指令减小、提前减速),
        # 才是真阻尼;写成 -kd*doff 反而助推过冲(高动量交接时尤其明显)。
        raw = (cfg.kp * e[0] + cfg.ki * st.integ[0] + cfg.kd * doff[0],
               cfg.kp * e[1] + cfg.ki * st.integ[1] + cfg.kd * doff[1])
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
