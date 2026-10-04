"""阶梯式降高 IBVS:粗接近世界坐标 PBVS(把筒弄进视野)-> 末端像素 IBVS 精对准
(对定位误差免疫);降高节奏是"收敛才降、超差停降"的阶梯——名字即此特征(对照:
锥面约束连续降高的 cone_descent_ibvs)。决策见
docs/spec/adr/0005-terminal-align-visual-servoing.md;组织/迁移约定见
cuadc_decision/README.md(一个 approach 一个模块 + 同名 sim/real yaml 跟着放)。

纯 Python,不依赖 rclpy——可脱离节点单测(含末端 IBVS 控制律)。approach 只消费
MissionContext(遥测 + 检测),吐 ApproachOutput,不碰 ROS 类型。可调参数(增益/
门限/降高率/投放容差…)由 `<此目录>/<NAME>.{sim,real}.yaml` 提供,`load_config`
读取,场景启动脚本选 profile。
"""

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple

import yaml

from cuadc_decision.approaches.base import (
    ApproachOutput, airframe_fields, bearing_to_ground_offset, clip as _clip, nearest_within as _nearest_within)
from cuadc_decision.drop_strategy import Candidate

NAME = 'stepped_descent_ibvs'

_OFF_WINDOW = 3  # 冻结偏移取近几帧实测均值(0.3s@10Hz):单帧噪声不毒化 coast/收口判据
_SERVO_LP = 2    # 伺服误差取近 2 帧均值:压检测噪声进 PID 的抖动;等效滞后 ~0.05s,远小于回路死区,不伤带宽


@dataclass
class ApproachConfig:
    """approach 的可调参数。字段名 = yaml 键。默认值只是兜底(纯单测/import 用),
    正常从 <NAME>.<profile>.yaml 读:sim 是 Gazebo 验证值,real 上真机按真实
    延迟/动力学重调。"""
    cruise_alt_m: float = 2.0
    drop_alt_m: float = 1.0               # 停降高度,h(EKF 解锁点参照)计;飞控离地=此值+fc_rest_clearance_m
    cylinder_height_m: float = 0.3        # 圆筒高;相机到筒顶深度见 _bearing_to_off
    # h 是飞控相对解锁点(EKF 原点)的高度,不是离地高:解锁时飞控已离地 fc_rest_clearance_m。
    # 相机又在飞控下方 camera_below_fc_m。二者都为 0 时退化为"h=相机离地"(纯单测兜底)。
    fc_rest_clearance_m: float = 0.0      # 飞控静止离地高，load_config 从 cuadc_airframe 注入
    camera_below_fc_m: float = 0.0        # 相机在飞控下方，同上注入
    dt_s: float = 0.1                     # tick 周期(对应 decision tick_rate_hz=10),积分/微分用
    min_conf: float = 0.5                 # 粗段/锁定:只对置信度够的检测伺服
    # 末端置信度门单独放低:近距圆盘贴画面边缘残缺,检测器置信度塌落(实测 h<1.1 时
    # near_det=0.01 仍被 0.5 门滤掉->丢筒)。末端有世界坐标硬锁关联门(refine_gate)
    # 保护,低置信度的"对的筒"照常伺服;垃圾检测靠关联门挡。
    min_conf_terminal: float = 0.5
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
    # 抛物线刹车:速度模长再限 √(2·a·herr)——大误差时也只给"以 a 减速刚好能停在目标"
    # 的速度,天然无过冲(位置伺服标准做法);0=关(退化为纯 v_max 限幅)。
    brake_decel_mps2: float = 0.0
    i_zone_m: float = 0.4                 # 只在近端积分,防大接近段积分饱和回弹
    # 投放口杆臂表,机体系(前,右)米=投放口相对相机,按投放次序排:第 N 次投放用第 N 项。
    # load_config 从 cuadc_airframe 注入(投放口位置 − 相机位置)。enter 时按
    # ctx.drop_task.drop_count 选定本次杆臂,超出表长用最后一项。
    lever_arms_m: Tuple[Tuple[float, float], ...] = ((0.0, 0.0),)
    # 经验瞄准修正,机体系(前,右)米,按投放次序与 lever_arms_m 并行(超表长用最后一项)。
    # 吸收解析链吸收不掉的系统误差(安装角/内参/杆臂测量/深度残差):标定流程=实测落点
    # 偏差取均值反填(落点比筒心偏机体前方 5cm -> 前向 trim -0.05)。生效值 lever+trim。
    aim_trims_m: Tuple[Tuple[float, float], ...] = ((0.0, 0.0),)
    max_coast_ticks: int = 4              # 末段连续失配超过此帧数就悬停,不靠陈旧偏移乱冲
    # 投放窗收口锁:末端贴近时筒必然滑出窄 FOV(杆臂瞄点在画面下缘,真机同几何)。丢检测
    # 时若已收敛(最后实测误差<drop_tol)且 h 离 drop_alt 不超过此带,则不受 coast 上限
    # 约束,按冻结偏移把盲降走完并照常查投放门(live 速度/水平)。盲段有界(≤此带+稳住时长,
    # 指令趋零速),与"陈旧偏移乱冲"的跑飞场景不同;未收敛就丢仍悬停等重锁。0=关闭。
    endgame_alt_band_m: float = 0.0
    # 末端持续丢失(超过此 tick 数)后退回粗段重走漏斗(爬回巡航高、位置回锁定点上方,
    # 全部复用粗段机制)——低空丢定死锁(筒在窄 FOV 外、悬停永远等不到重锁)的唯一出路。
    # 0=关(纯单测兜底)。
    reacquire_after_ticks: int = 0
    # 降高
    descend_rate_mps: float = 0.4
    descend_pause_off_m: float = 0.30     # 水平误差大于此暂停降高、先追回来
    # 末端高度按比例收敛/保持:降高段 vz=min(rate, p·(h-drop_alt)) 平滑进入不超冲
    # (0.4m/s 阶跃停降实测冲过 0.2m 到 0.8,圆盘残缺质心偏移把投放门骗过->26cm 脱靶);
    # 投放段 h 低于 drop_alt 会按同增益爬回。0=关(退化为老的阶跃停降,纯单测兜底)。
    alt_hold_p: float = 0.0
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
    data.update(airframe_fields())
    return ApproachConfig(**data)


def create(config: Optional[ApproachConfig] = None) -> 'SteppedDescentIbvs':
    return SteppedDescentIbvs(config)


@dataclass
class ApproachState:
    phase: str = 'coarse'                 # coarse -> terminal (持续丢失会退回 coarse 重来)
    locked: Optional[Candidate] = None    # 锁定目标(世界坐标),粗/末段硬锁都用它关联
    lever: Tuple[float, float] = (0.0, 0.0)  # 本次投放口杆臂,enter 时按投放次序从 lever_arms_m 选定
    miss: int = 0                         # 末段连续失配帧数(丢检测),超阈值悬停
    integ: Tuple[float, float] = (0.0, 0.0)
    prev_off: Optional[Tuple[float, float]] = None       # 上一帧偏移,PID 的 D 项(on measurement)用
    prev_off_meas: Optional[Tuple[float, float]] = None  # 近几帧实测偏移均值(见 _OFF_WINDOW),coast/收口用
    off_window: list = field(default_factory=list)       # 实测偏移滑窗,冻结值抗单帧噪声(模式 A 毒帧)
    last_bearing: Optional[Tuple[float, float]] = None   # 最后实测 bearing,投放瞬间日志用(经验瞄点的可复现性诊断)
    stable_since: Optional[float] = None


class SteppedDescentIbvs:
    """粗接近用世界坐标 PBVS 把飞机带到目标附近(把筒弄进视野),末端切像素
    IBVS 精对准——对定位误差免疫(见 ADR-0005)。全程硬锁选定目标:粗/末段
    都按世界坐标最近门关联(姿态无关、能区分相邻筒),别的筒进视野不跳;末段
    伺服用 bearing 姿态补偿偏移(定位无关)——关联用世界、控制用 bearing。"""

    name = NAME

    def __init__(self, cfg: Optional[ApproachConfig] = None):
        self.cfg = cfg or ApproachConfig()
        self.state = ApproachState()

    def enter(self, ctx):
        """进入 APPROACH 时调:以 SEARCH 选定的目标为锁定种子,按投放次序选投放口杆臂
        (+经验修正),清内部状态。"""
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
            e = (off[0] - st.lever[0], off[1] - st.lever[1])
            settled = ctx.horizontal_speed is not None and ctx.horizontal_speed <= cfg.handoff_speed_mps
            if math.hypot(*e) <= cfg.handoff_off_m and settled:
                st.phase = 'terminal'
                st.miss, st.prev_off_meas, st.off_window = 0, off, [off]
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
        endgame = False
        if best is not None:
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
            # 投放窗收口锁(见 endgame_alt_band_m 注释):已收敛+已贴近才允许按冻结
            # 偏移盲收口,否则维持"悬停等重锁"。
            endgame = (st.prev_off_meas is not None
                       and math.hypot(st.prev_off_meas[0] - st.lever[0],
                                      st.prev_off_meas[1] - st.lever[1]) < cfg.drop_tol_m
                       and h <= cfg.drop_alt_m + cfg.endgame_alt_band_m)
            if st.prev_off_meas is None or (st.miss > cfg.max_coast_ticks and not endgame):
                # 彻底丢目标:悬停(零机体速度),绝不靠陈旧偏移继续冲(那正是之前
                # 跑飞的原因),清 PID 状态,等重新关联。
                st.integ, st.prev_off, st.stable_since = (0.0, 0.0), None, None
                if st.miss >= cfg.reacquire_after_ticks > 0:
                    # 低空重锁等不到(筒在窄 FOV 外):退回粗段重走漏斗。
                    st.phase, st.miss = 'coarse', 0
                    st.prev_off_meas, st.off_window = None, []
                    return self._coarse(ctx, dets, h)
                return ApproachOutput(velocity_target=(0.0, 0.0, 0.0))
            off = st.prev_off_meas  # 短暂丢失/收口:保持上次偏移

        e = (off[0] - st.lever[0], off[1] - st.lever[1])
        doff = (0.0, 0.0) if st.prev_off is None else (
            (off[0] - st.prev_off[0]) / cfg.dt_s, (off[1] - st.prev_off[1]) / cfg.dt_s)
        st.prev_off = off
        # 本控制器是"位置误差 e=off-lever -> 速度指令",不是常规"误差 -> 力"。
        # 接近目标时 off 减小、doff<0,阻尼项应 +kd*doff(使指令减小、提前减速),
        # 才是真阻尼;写成 -kd*doff 反而助推过冲(高动量交接时尤其明显)。
        raw = (cfg.kp * e[0] + cfg.ki * st.integ[0] + cfg.kd * doff[0],
               cfg.kp * e[1] + cfg.ki * st.integ[1] + cfg.kd * doff[1])
        herr = math.hypot(*e)
        v_lim = cfg.v_max_mps
        if cfg.brake_decel_mps2 > 0.0:  # 抛物线刹车(见 brake_decel_mps2 注释)
            v_lim = min(v_lim, math.sqrt(2.0 * cfg.brake_decel_mps2 * herr))
        mag = math.hypot(*raw)
        scale = min(1.0, v_lim / mag) if mag > 1e-9 else 0.0  # 模长限幅,保方向
        v = (raw[0] * scale, raw[1] * scale)
        integ = list(st.integ)
        for i in (0, 1):  # 条件积分:近端且未饱和才积分
            if abs(raw[i]) < v_lim and abs(e[i]) < cfg.i_zone_m:
                integ[i] += e[i] * cfg.dt_s
        st.integ = (integ[0], integ[1])
        if h > cfg.drop_alt_m + 1e-3:  # 还在降高:够居中才降,飘了先追回来
            # 降高只许在有新测量或收口锁内进行:普通 coast 盲降会把丢失点的高度
            # 亏空扩大(实测 4 帧 coast 从 1.05 带到 0.9 以下),丢了就先停降。
            # 收口带内降高门收紧到 drop_tol:低于带顶随时可能永久丢筒(FOV 几何),
            # 必须保证"丢的那一刻已收敛",收口锁才接得住;带外维持宽门快速下来。
            in_band = h <= cfg.drop_alt_m + cfg.endgame_alt_band_m
            gate = cfg.drop_tol_m if in_band else cfg.descend_pause_off_m
            rate = cfg.descend_rate_mps
            if cfg.alt_hold_p > 0.0:  # 比例减速进入 drop_alt,不超冲(见 alt_hold_p 注释)
                rate = min(rate, cfg.alt_hold_p * (h - cfg.drop_alt_m))
            vz = rate if herr < gate and (st.miss == 0 or endgame) else 0.0
            return ApproachOutput(velocity_target=(v[0], v[1], vz))

        # 到投放高度:高度 P 保持(超冲/下沉会爬回 drop_alt——过低时圆盘残缺、
        # 质心量测有偏,必须回到量测可信的高度),检查投放门(用对地遥测速度)
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

    def _bearing_to_off(self, bearing, ctx, h):
        """bearing -> 相对正下方的水平偏移(几何在 base.bearing_to_ground_offset)。
        姿态/高度优先取拍照时刻那套(ctx.det_*,与 bearing 同刻,ADR-0010 B);缺失退化
        用实时遥测。深度=相机离地(h+静置离地−相机下沉)−筒高:h 是 EKF 解锁点参照,
        基准错任何一层,偏心杆臂就系统性偏。"""
        roll = (ctx.det_roll_rad if ctx.det_roll_rad is not None else ctx.roll_rad) or 0.0
        pitch = (ctx.det_pitch_rad if ctx.det_pitch_rad is not None else ctx.pitch_rad) or 0.0
        if ctx.det_height_m is not None:
            h = ctx.det_height_m
        cam_agl = h + self.cfg.fc_rest_clearance_m - self.cfg.camera_below_fc_m
        return bearing_to_ground_offset(bearing, roll, pitch, cam_agl - self.cfg.cylinder_height_m)
