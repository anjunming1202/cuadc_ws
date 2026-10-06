"""决策层的任务状态机与各阶段逻辑，纯 Python，不依赖 rclpy——可以脱离
ROS2 节点单独单元测试。状态机本身、阶段划分、跨状态强制打断的设计
依据见 docs/spec/06-planning.md；感知消息契约见
docs/spec/05-perception-decision-interface.md。

`decision_node.py` 是唯一碰 ROS2 API 的地方：它把订阅到的遥测/检测
组装成 `MissionContext`，每个 tick 调用这里的 `tick()`，再把返回的
`TickResult` 翻译成真正的 service 调用/话题发布。这里的函数只接收/
返回普通数据，不认识任何 ROS 消息类型。

"tick"：decision_node.py 起一个定时器（tick_rate_hz），每次触发调一次
`tick()`，类似游戏循环的一帧——"当前状态+最新遥测/检测，这一轮该做
什么"。每个状态自己的 `_tick_xxx(ctx)` 就是"处于这个状态时，一个 tick
该做的事"。

参数分三处、按属主分开（见 docs/spec/mission-config-layer.md）：世界/任务/场景参数
（区中心、扫描航点、一般巡航高度、要投几次、跑哪些阶段…）是新的**单一真源配置层**
`mission/`（sim/real 分文件），由 decision_node 载入后经 `configure_mission()` 注入这里；
本 FSM 本体的通用调参（到点容差/阶段超时/安全守卫）在挨着本文件的
`fsm_tuning.{sim,real}.yaml`（`FsmTuning`/`load_tuning`）；SEARCH 的聚类/选目标算法
及其参数在 `search_strategy.py`（+ 同名 yaml）；投放目标选哪个在 `drop_strategy.py`；
接近算法在 `approaches/`；侦察区怎么覆盖在 `surveys/`。本文件不再持有这些为模块级常量。
"""

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import cuadc_physical
import yaml

from cuadc_decision import approaches
from cuadc_decision import drop_strategy
from cuadc_decision import search_strategy
from cuadc_decision import surveys
from cuadc_decision.drop_strategy import Candidate
from cuadc_decision.mission import config as mission_config


class State:
    """扁平状态机的状态名，见 docs/spec/06-planning.md。"""

    INIT = 'INIT'
    WAIT_VEHICLE_READY = 'WAIT_VEHICLE_READY'
    WAIT_TASK_START = 'WAIT_TASK_START'
    WAIT_PILOT_ARM = 'WAIT_PILOT_ARM'
    TAKEOFF = 'TAKEOFF'
    NAVIGATE_TO_DROP_ZONE = 'NAVIGATE_TO_DROP_ZONE'
    SEARCH_DROP_TARGETS = 'SEARCH_DROP_TARGETS'
    APPROACH_DROP_TARGET = 'APPROACH_DROP_TARGET'
    DROP_PAYLOAD = 'DROP_PAYLOAD'
    NAVIGATE_TO_RECON_ZONE = 'NAVIGATE_TO_RECON_ZONE'
    SURVEY_RECON_ZONE = 'SURVEY_RECON_ZONE'
    RTL_OR_LAND = 'RTL_OR_LAND'
    PILOT_TAKEOVER = 'PILOT_TAKEOVER'


# 解锁/上锁归飞手（Pi 不发 arm）：飞手须在此模式下 RC 解锁，开关在别的挡会顶掉 GUIDED。
_ARM_MODE = 'STABILIZE'
# Pi 接管后自己会设的模式；遥测出现这之外的模式 = 飞手拨开关或飞控 failsafe 接手。
_PI_MODES = frozenset({'GUIDED', 'RTL'})


@dataclass
class FsmTuning:
    """顶层任务 FSM 本体的通用调参（B′）——不属于任何可替换算法，是这个唯一的状态机
    自己的机制。字段名 = `fsm_tuning.<profile>.yaml` 的键。默认值只是兜底（纯单测/
    import 期不读文件），正常由 decision_node `load_tuning(profile)` 读入后注入。"""
    position_tolerance_m: float = 0.5
    altitude_tolerance_m: float = 0.5
    recon_coverage_match_m: float = 0.6       # 侦察覆盖去重（surveys/detect_count）：新检测离已覆盖点多近算同一个筒
    drop_phase_timeout_s: float = 120.0
    recon_phase_timeout_s: float = 90.0
    battery_critical_fraction: float = 0.20   # sensor_msgs/BatteryState.percentage 是 0~1
    gps_fix_ok_threshold: int = 3             # MAVLink GPS_FIX_TYPE：3 = 3D fix
    emulate_pilot_arm: bool = False           # 仅 sim：没有飞手，WAIT_PILOT_ARM 里替飞手切 STABILIZE+解锁
    transit_speed_mps: float = 5.0            # 飞往投放区/侦察区的水平限速
    search_speed_mps: float = 3.0             # 搜索/侦察航点之间、接近粗对准的水平限速；飞控 WP_SPD 设同值兜底
    zone_climb_mps: float = 0.0               # 投放/侦察区内爬升限速复写；0 = 用飞控 WP_SPD_UP
    zone_descent_mps: float = 0.0             # 投放/侦察区内下降限速复写；0 = 用飞控 WP_SPD_DN
    # 失效处理阈值，全部失效路径见 docs/spec/failure-handling.md
    takeoff_timeout_s: float = 20.0           # 起飞爬不到巡航高度 -> RTL
    nav_timeout_margin_s: float = 10.0        # 转场每段超时 = 2×距离/转场速度 + 此余量，到时直接进区作业
    search_rescans: int = 1                   # 扫完没有可选目标时重扫的轮数，用完放弃投放阶段
    release_max_retries: int = 2              # release_payload 失败重试次数，用完记为已尝试、继续
    perception_stale_log_s: float = 3.0       # 作业中感知断流超过此时长记一条 FAULT（只记录，不改任务逻辑）


def load_tuning(profile: str) -> FsmTuning:
    """从挨着本文件的 `fsm_tuning.<profile>.yaml` 读 FSM 本体调参。"""
    path = Path(__file__).parent / f'fsm_tuning.{profile}.yaml'
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    return FsmTuning(**data)


def _scan_waypoints(center, offsets, takeoff_point):
    """场景配置里的区中心 + 相对 offsets（场地坐标）算成起飞坐标系扫描航点列表。"""
    return [mission_config.field_to_takeoff(center[0] + dx, center[1] + dy, takeoff_point) for dx, dy in offsets]


_DROP_STATES = frozenset({State.SEARCH_DROP_TARGETS, State.APPROACH_DROP_TARGET, State.DROP_PAYLOAD})
_RECON_STATES = frozenset({State.SURVEY_RECON_ZONE})

# ---- 注入式活动配置（见文件头）。import 期用各自 dataclass 默认建一套（纯单测/import
# 不读文件）；decision_node 起来后按 profile/scenario 调 configure_*() 换成对应档。----
_WORLD = mission_config.MissionWorld()          # A：世界/任务/场景
_DROP_PORTS = len(cuadc_physical.load_airframe().drop_ports)
_TUNING = FsmTuning()                            # B′：FSM 本体调参
_SEARCH = search_strategy.SearchConfig()         # B：SEARCH 算法参数
_DROP_SCAN_WAYPOINTS = _scan_waypoints(_WORLD.drop_zone_center, _WORLD.drop_scan_offsets,
                                       _WORLD.takeoff_point)
_ACTIVE_SURVEY = surveys.create_active(_WORLD, _TUNING)  # 侦察区覆盖（见 surveys/），configure_mission 换档

# 可替换的接近阶段策略实例（粗 PBVS -> 末端 IBVS 等，见 approaches/）。持有跨 tick 的接近
# 内部状态（锁定目标）;进入 APPROACH 时 _tick_search 调 .enter() 重置。换算法改
# approaches.ACTIVE，不动状态机。可调参数在该 approach 的 sim/real yaml。
_ACTIVE_APPROACH = approaches.create_active()


def configure_approach(profile, name=None, variant=None):
    """按 sim/real profile 从 yaml 载入 approach（name 缺省 = approaches.ACTIVE，variant 叠加
    <name>.<variant>.yaml）,重建活动实例。decision_node 在 __init__ 里调一次。"""
    global _ACTIVE_APPROACH
    _ACTIVE_APPROACH = approaches.create_active(profile=profile, name=name, variant=variant)


def configure_mission(world, tuning, search, survey=None):
    """注入世界/任务配置(A)、FSM 本体调参(B′)、SEARCH 算法配置(B)、侦察覆盖实例（缺省 = surveys.ACTIVE
    按 dataclass 默认，跟这次的 world 建），并按 world 里的区中心/offsets 重算起飞坐标系扫描航点。
    decision_node 在 __init__ 里调一次。"""
    global _WORLD, _TUNING, _SEARCH, _DROP_SCAN_WAYPOINTS, _ACTIVE_SURVEY
    _WORLD, _TUNING, _SEARCH = world, tuning, search
    _DROP_SCAN_WAYPOINTS = _scan_waypoints(
        world.drop_zone_center, world.drop_scan_offsets, world.takeoff_point)
    _ACTIVE_SURVEY = survey or surveys.create_active(world, tuning)


# ---- decision_node.py 与这里交换数据用的普通数据类型（不含任何 ROS 类型）----

@dataclass
class ServiceCallRequest:
    name: str
    kwargs: dict


@dataclass
class ServiceResult:
    name: str
    success: bool


@dataclass
class Fault:
    """一次失效/异常处理事件：decision_node 打 [FAULT] WARN 并发到 ~/fault_events。代码表见
    docs/spec/failure-handling.md。"""
    code: str
    reason: str


@dataclass
class TickResult:
    next_state: Optional[str] = None
    position_target: Optional[Tuple[float, float, float]] = None  # (x, y, z)，同 telemetry/local_position 坐标系
    velocity_target: Optional[Tuple[float, float, float]] = None  # (前,右,下) 机体系速度 m/s，末端 IBVS 用，见 04-interfaces.md cmd/velocity
    speed_target_mps: Optional[float] = None  # 位置类移动的水平限速，见 04-interfaces.md cmd/speed
    climb_speed_mps: Optional[float] = None   # 爬升限速复写（与 descent 至多一个），见 cmd/climb_speed
    descent_speed_mps: Optional[float] = None  # 下降限速复写，见 cmd/descent_speed
    service_call: Optional[ServiceCallRequest] = None
    notice: Optional[str] = None  # 给现场操作员看的提示（decision_node 打 WARN），只在状况变化时给
    fault: Optional[Fault] = None


@dataclass
class DropTaskState:
    clusters: List[list] = field(default_factory=list)  # 每个元素是 [(x, y, size), ...] 采样列表
    used_positions: List[Tuple[float, float]] = field(default_factory=list)
    drop_count: int = 0
    selected_target: Optional[Candidate] = None
    phase_entered_monotonic: Optional[float] = None
    waypoint_index: int = 0
    rejected_positions: List[Tuple[float, float]] = field(default_factory=list)  # 放弃过的目标，不再选/不再改锁
    rescans: int = 0                          # 本轮找目标已重扫次数，选中目标即清零
    release_failures: int = 0                 # 本次投放 release_payload 失败次数


@dataclass
class ReconTaskState:
    phase_entered_monotonic: Optional[float] = None  # 覆盖进度归 surveys/ 实例自己持有


@dataclass
class MissionContext:
    """一个 tick 需要知道的全部东西。`decision_node.py` 每个 tick 用
    订阅到的最新遥测/检测更新这些字段，再传给 `tick()`。"""

    current_state: str = State.INIT
    state_entered_monotonic: float = field(default_factory=time.monotonic)
    command_dispatched: bool = False
    last_service_result: Optional[ServiceResult] = None
    takeoff_step: str = 'set_mode'
    pilot_arm_step: str = 'set_mode'          # 仅 emulate_pilot_arm（sim）用
    pilot_arm_ready: bool = False             # 进 WAIT_PILOT_ARM 后已看到过未解锁（下一次解锁才算数）
    already_armed_noted: bool = False
    pi_in_control: bool = False               # 起飞指令被接受起 = Pi 接管；之后飞手改模式/上锁即夺控
    drop_port_start: int = 0                  # 第一投用的投放口（0 左 / 1 右），之后轮换
    rtl_commanded: bool = False               # RTL_OR_LAND 已切过 RTL（或飞控已在 RTL）：不再重发

    connected: bool = False
    armed: bool = False
    mode: Optional[str] = None
    gps_fix_type: int = 0
    ekf_healthy: bool = False
    battery_fraction: Optional[float] = None
    failsafe_active: bool = False
    local_position_xy: Optional[Tuple[float, float]] = None
    local_position_z: Optional[float] = None  # 起飞坐标系 z，向下为正（高度 = -z）
    horizontal_speed: Optional[float] = None  # 水平合速度，米/秒
    vertical_speed: Optional[float] = None  # 起飞坐标系 vz，向下为正，米/秒
    roll_rad: Optional[float] = None
    pitch_rad: Optional[float] = None
    yaw_rad: Optional[float] = None          # 起飞坐标系航向（0=起飞时机头方向，向右转为正）
    detections: list = field(default_factory=list)  # 元素需要有 .target_type/.position.x/.position.y/.apparent_size
    # 当前 detections 那帧图像**拍照时刻**的姿态/离地高(来自 DetectionArray.vehicle_pose)。
    # IBVS 姿态补偿/米换算必须用它(与 bearing 同刻)；投放门等"当下状态"判断仍用实时遥测。
    det_roll_rad: Optional[float] = None
    det_pitch_rad: Optional[float] = None
    det_height_m: Optional[float] = None
    task_start_triggered: bool = False
    takeoff_wait_since: Optional[float] = None   # 起飞指令被接受、开始等爬升的时刻
    nav_timeout_key: Optional[float] = None      # 当前转场段的超时按进入该状态时刻算一次
    nav_timeout_s: float = 0.0

    drop_task: DropTaskState = field(default_factory=DropTaskState)
    recon_task: ReconTaskState = field(default_factory=ReconTaskState)

    @property
    def drop_port(self) -> int:
        """本投用的投放口号：从 drop_port_start 起轮换。瞄准（杠杆臂）和放料共用这一个值。"""
        return (self.drop_task.drop_count + self.drop_port_start) % _DROP_PORTS


# ---- 通用小工具 ----

def _arrived(current_xy, target_xy, tolerance):
    if current_xy is None:
        return False
    return math.hypot(current_xy[0] - target_xy[0], current_xy[1] - target_xy[1]) <= tolerance


def _at_scan_waypoint(ctx, waypoint):
    """扫描航点要水平到点且到了搜索高度才算到：从过渡高度爬升途中不推进航点，保证按 search_alt_m 的覆盖扫。"""
    return (_arrived(ctx.local_position_xy, waypoint, _TUNING.position_tolerance_m)
            and ctx.local_position_z is not None
            and abs(-ctx.local_position_z - _WORLD.search_alt_m) <= _TUNING.altitude_tolerance_m)


def _zone_speeds(ctx, target_z):
    """投放/侦察区内位置类移动的限速。飞控一次只认一个竖直复写（设爬升会把下降打回默认），只发当前升降方向那个。"""
    speeds = {'speed_target_mps': _TUNING.search_speed_mps}
    if ctx.local_position_z is not None:
        if target_z < ctx.local_position_z - 0.05 and _TUNING.zone_climb_mps > 0.0:
            speeds['climb_speed_mps'] = _TUNING.zone_climb_mps
        elif target_z > ctx.local_position_z + 0.05 and _TUNING.zone_descent_mps > 0.0:
            speeds['descent_speed_mps'] = _TUNING.zone_descent_mps
    return speeds


def _nav_timed_out(ctx, target_xy):
    """转场段超时：进入该状态时按 2×距离/转场速度 + 余量定一次时限。"""
    if ctx.nav_timeout_key != ctx.state_entered_monotonic:
        ctx.nav_timeout_key = ctx.state_entered_monotonic
        dist = (math.hypot(ctx.local_position_xy[0] - target_xy[0], ctx.local_position_xy[1] - target_xy[1])
                if ctx.local_position_xy is not None else 0.0)
        ctx.nav_timeout_s = 2.0 * dist / _TUNING.transit_speed_mps + _TUNING.nav_timeout_margin_s
    return time.monotonic() - ctx.state_entered_monotonic > ctx.nav_timeout_s


def _excluded(ctx):
    """不再作为目标的位置：已投过的 + 放弃过的。"""
    return ctx.drop_task.used_positions + ctx.drop_task.rejected_positions


def _leave_drop_phase():
    """放弃/结束投放阶段的去向由场景 phases 决定（同阶段超时守卫）。"""
    return State.NAVIGATE_TO_RECON_ZONE if 'recon' in _WORLD.phases else State.RTL_OR_LAND


def _after_takeoff():
    """起飞后去第一个要跑的阶段：phases 不含 drop（侦察单练）就直接去侦察。"""
    return State.NAVIGATE_TO_DROP_ZONE if 'drop' in _WORLD.phases else _leave_drop_phase()


def _start_approach(ctx, target):
    ctx.drop_task.selected_target = target
    ctx.drop_task.rescans = 0
    _ACTIVE_APPROACH.enter(ctx)  # 以选定目标为锁定种子，清接近内部状态
    return TickResult(next_state=State.APPROACH_DROP_TARGET)


def _next_target_or_search(ctx, fault=None):
    """投完一个/放弃一个目标后：先用已有聚类里的下一个候选直接粗对准（不再整轮搜索）；
    没有可选候选才回 SEARCH 从头扫。"""
    target = drop_strategy.ACTIVE_STRATEGY(
        search_strategy.candidates(ctx, _SEARCH), _excluded(ctx), ctx.drop_task.drop_count)
    if target is not None:
        result = _start_approach(ctx, target)
    else:
        ctx.drop_task.waypoint_index = 0
        result = TickResult(next_state=State.SEARCH_DROP_TARGETS)
    result.fault = fault
    return result


def _dispatch_and_await(ctx, service_name, kwargs, on_success):
    """一次性 service 调用的通用模式：发起一次、等结果，成功就跑
    `on_success()` 拿下一步动作，失败就重试。`set_mode`/`arm`/
    `takeoff`/`release_payload` 四处一次性调用共用这段逻辑，不各写一遍。"""
    if ctx.last_service_result is not None and ctx.last_service_result.name == service_name:
        ctx.command_dispatched = False
        if ctx.last_service_result.success:
            return on_success()
        # 失败：不返回，往下走会重新发起

    if not ctx.command_dispatched:
        ctx.command_dispatched = True
        return TickResult(service_call=ServiceCallRequest(service_name, kwargs))
    return TickResult()


# ---- 各状态的 tick 逻辑 ----
# 投放目标选哪个见 drop_strategy.py；SEARCH 的聚类/候选/选目标门见 search_strategy.py。

def _tick_init(ctx):
    return TickResult(next_state=State.WAIT_VEHICLE_READY)


def _tick_wait_vehicle_ready(ctx):
    # EKF/GPS 正常，用作 PreArm 通过的替代判据——接口层契约里没有单独
    # 暴露 PreArm 状态字段，这是目前能拿到的最接近的信号。不看
    # ctx.connected：接口层的 status/connected 只在刚连上/断线那一刻
    # 各发一次，不是持续重发的状态，决策节点如果比接口层晚起、错过那
    # 一次 True 就永远等不到；ekf_healthy/gps_fix_type 默认值本身就是
    # "还没收到过消息"的状态（False/0），已经隐含了"至少收到过一条
    # 遥测"这个前提，不需要再单独判断 connected。
    if ctx.ekf_healthy and ctx.gps_fix_type >= _TUNING.gps_fix_ok_threshold:
        return TickResult(next_state=State.WAIT_TASK_START)
    return TickResult()


def _tick_wait_task_start(ctx):
    # 真机上这里应该是读物理 GPIO 任务开关（见 06-planning.md），仿真/
    # 开发环境没有真实 GPIO，用 decision_node 订阅的一个替代触发话题
    # 代替，见该文件里的说明。
    if ctx.task_start_triggered:
        return TickResult(next_state=State.WAIT_PILOT_ARM,
                          notice=f'task started — WAITING FOR PILOT ARM ({_ARM_MODE} + RC arm)')
    return TickResult()


def _emulate_pilot_arm(ctx):
    # 仅 sim：没有飞手，替飞手切 STABILIZE + 解锁；之后的判定（模式、接管）与真机完全相同。
    if ctx.pilot_arm_step == 'set_mode':
        def on_success():
            ctx.pilot_arm_step = 'arm'
            return TickResult()
        return _dispatch_and_await(ctx, 'set_mode', {'mode': _ARM_MODE}, on_success)
    if ctx.pilot_arm_step == 'arm':
        def on_success():
            ctx.pilot_arm_step = 'done'
            return TickResult()
        return _dispatch_and_await(ctx, 'arm', {'arm': True}, on_success)
    return TickResult()


def _tick_wait_pilot_arm(ctx):
    # 起飞的最后一个动作永远是飞手 RC 解锁：本状态里先看到未解锁、再看到解锁才算数
    # （放行前就解锁的不算）。Pi 看到解锁立刻起飞，赶在飞控 DISARM_DELAY 自动上锁之前。
    if not ctx.armed:
        ctx.pilot_arm_ready = True
        ctx.already_armed_noted = False
        if _TUNING.emulate_pilot_arm:
            return _emulate_pilot_arm(ctx)
        return TickResult()
    if not ctx.pilot_arm_ready:
        if not ctx.already_armed_noted:
            ctx.already_armed_noted = True
            return TickResult(notice='vehicle ALREADY ARMED — pilot: disarm with RC, '
                                     'then re-arm when ready (only a fresh arm starts the flight)')
        return TickResult()
    if ctx.mode != _ARM_MODE:
        ctx.pilot_arm_ready = False
        ctx.already_armed_noted = True
        return TickResult(notice=f'armed in {ctx.mode}, expected {_ARM_MODE} — NOT taking off; '
                                 f'pilot: disarm, mode switch to {_ARM_MODE}, re-arm')
    ctx.takeoff_step = 'set_mode'
    return TickResult(next_state=State.TAKEOFF,
                      notice=f'pilot armed in {_ARM_MODE} -> Pi taking control')


def _abort_takeoff(ctx, why):
    # 离地前飞手上锁/改模式：放弃这次起飞，回去等飞手重新解锁（放行信号保持有效）。
    ctx.takeoff_step = 'set_mode'
    ctx.pilot_arm_step = 'set_mode'
    ctx.pilot_arm_ready = False
    return TickResult(next_state=State.WAIT_PILOT_ARM, notice=f'takeoff aborted: {why}',
                      fault=Fault('TAKEOFF_ABORTED', why))


def _tick_takeoff(ctx):
    step = ctx.takeoff_step

    if step != 'wait_altitude' and not ctx.armed:
        return _abort_takeoff(ctx, 'pilot disarmed before liftoff')

    if step == 'set_mode':
        def on_success():
            ctx.takeoff_step = 'wait_guided'
            return TickResult()
        return _dispatch_and_await(ctx, 'set_mode', {'mode': 'GUIDED'}, on_success)

    if step == 'wait_guided':
        # 等 GUIDED 的遥测真到了再发起飞（set_mode 的应答早于下一条心跳）。
        if ctx.mode == 'GUIDED':
            ctx.takeoff_step = 'takeoff'
        return TickResult()

    if step == 'takeoff':
        if ctx.mode != 'GUIDED':
            return _abort_takeoff(ctx, f'mode changed to {ctx.mode} before liftoff')

        def on_success():
            ctx.takeoff_step = 'wait_altitude'
            ctx.takeoff_wait_since = time.monotonic()
            ctx.pi_in_control = True
            return TickResult()
        return _dispatch_and_await(ctx, 'takeoff', {'alt': _WORLD.cruise_alt_m}, on_success)

    # step == 'wait_altitude'：takeoff service 只确认指令被接受，不代表
    # 已经爬升到位，得自己比对遥测（04-interfaces.md 的边界约定）。
    if ctx.local_position_z is not None and -ctx.local_position_z >= _WORLD.cruise_alt_m - _TUNING.altitude_tolerance_m:
        return TickResult(next_state=_after_takeoff())
    if ctx.takeoff_wait_since is not None and time.monotonic() - ctx.takeoff_wait_since > _TUNING.takeoff_timeout_s:
        h = -ctx.local_position_z if ctx.local_position_z is not None else float('nan')
        return TickResult(next_state=State.RTL_OR_LAND, fault=Fault(
            'TAKEOFF_TIMEOUT', f'h={h:.2f}m < {_WORLD.cruise_alt_m}m after {_TUNING.takeoff_timeout_s:.0f}s'))
    return TickResult()


def _zone_entry(first_waypoint, center):
    """转场终点：区中线上、与第一个扫描航点同一 x——沿中线直飞进区，到了再横移去扫描起点。"""
    return (first_waypoint[0], mission_config.field_to_takeoff(center[0], center[1], _WORLD.takeoff_point)[1])


def _tick_navigate_to_drop_zone(ctx):
    entry = _zone_entry(_DROP_SCAN_WAYPOINTS[0], _WORLD.drop_zone_center)
    arrived = _arrived(ctx.local_position_xy, entry, _TUNING.position_tolerance_m)
    if arrived or _nav_timed_out(ctx, entry):
        ctx.drop_task.phase_entered_monotonic = time.monotonic()
        fault = None if arrived else Fault(
            'NAV_TIMEOUT', f'not within {_TUNING.position_tolerance_m}m of {entry} after '
                           f'{ctx.nav_timeout_s:.0f}s -> start SEARCH anyway')
        return TickResult(next_state=State.SEARCH_DROP_TARGETS, fault=fault)
    return TickResult(position_target=(entry[0], entry[1], -_WORLD.cruise_alt_m),
                      speed_target_mps=_TUNING.transit_speed_mps)


def _tick_search_drop_targets(ctx):
    # 逐帧检测的过滤/时序聚类/候选提取/选目标门都在 search_strategy（可替换的 SEARCH
    # 算法）。这里只管：喂本帧检测、按扫描航点飞、把选中的目标交给接近阶段。只看投放区
    # 那一侧的检测：检测点换算到场地 x，小于 drop_recon_split_x_m（field-layout.md：投放区/
    # 侦察区中间隔 20m 过渡区，取中点粗分即可）。
    search_strategy.observe(
        ctx, _SEARCH,
        lambda x, _y: x + _WORLD.takeoff_point[0] < _WORLD.drop_recon_split_x_m)

    # 候选还不够/还没扫完，按扫描航点继续飞，攒更多样本
    index = min(ctx.drop_task.waypoint_index, len(_DROP_SCAN_WAYPOINTS) - 1)
    waypoint = _DROP_SCAN_WAYPOINTS[index]
    scan_complete = index == len(_DROP_SCAN_WAYPOINTS) - 1 and _at_scan_waypoint(ctx, waypoint)
    if _at_scan_waypoint(ctx, waypoint):
        if index < len(_DROP_SCAN_WAYPOINTS) - 1:
            ctx.drop_task.waypoint_index = index + 1
            waypoint = _DROP_SCAN_WAYPOINTS[ctx.drop_task.waypoint_index]

    target = search_strategy.select_target(
        ctx, _SEARCH, scan_complete, time.monotonic(), drop_strategy.ACTIVE_STRATEGY, _excluded(ctx))
    if target is not None:
        return _start_approach(ctx, target)
    if scan_complete:
        # 扫完一整轮仍没有可选目标（没看到 / 全部已投或已放弃）：重扫，次数用完放弃投放阶段。
        n = len(search_strategy.candidates(ctx, _SEARCH))
        if ctx.drop_task.rescans < _TUNING.search_rescans:
            ctx.drop_task.rescans += 1
            ctx.drop_task.waypoint_index = 0
            return TickResult(fault=Fault(
                'SEARCH_EMPTY_RESCAN', f'scan complete, no selectable target ({n} candidates, '
                                       f'{len(_excluded(ctx))} excluded) -> rescan {ctx.drop_task.rescans}'))
        return TickResult(next_state=_leave_drop_phase(), fault=Fault(
            'SEARCH_EMPTY_GIVE_UP', f'no selectable target after {ctx.drop_task.rescans} rescan(s), '
                                    f'drops={ctx.drop_task.drop_count} -> leave drop phase'))

    return TickResult(position_target=(waypoint[0], waypoint[1], -_WORLD.search_alt_m),
                      **_zone_speeds(ctx, -_WORLD.search_alt_m))


def _tick_approach_drop_target(ctx):
    # 接近的内部实现是可替换的 approach 单元（粗 PBVS -> 末端 IBVS 等），见
    # approaches/ / cuadc_decision/README.md。这里只把它的输出翻成 TickResult：
    # 对齐并稳住（drop_ready）就转 DROP，否则发它要的运动指令（粗段位置、末端速度）。
    # 放弃（give_up）= 该目标确认不了/对不上：拉黑，换已知的下一个候选或回 SEARCH。
    out = _ACTIVE_APPROACH.loop(ctx)
    event = Fault(*out.event) if out.event is not None else None
    if out.give_up is not None:
        target = ctx.drop_task.selected_target
        ctx.drop_task.rejected_positions.append((target.x, target.y))
        ctx.drop_task.selected_target = None
        return _next_target_or_search(ctx, Fault(
            out.give_up[0], f'{out.give_up[1]}; target ({target.x:.2f},{target.y:.2f}) rejected'))
    if out.drop_ready:
        return TickResult(next_state=State.DROP_PAYLOAD, fault=event)
    speeds = _zone_speeds(ctx, out.position_target[2]) if out.position_target is not None else {}
    if out.speed_mps is not None and out.position_target is not None:
        speeds['speed_target_mps'] = out.speed_mps
    return TickResult(position_target=out.position_target, velocity_target=out.velocity_target,
                      fault=event, **speeds)


def _tick_drop_payload(ctx):
    def finish(fault=None):
        target = ctx.drop_task.selected_target
        ctx.drop_task.used_positions.append((target.x, target.y))
        ctx.drop_task.drop_count += 1
        ctx.drop_task.selected_target = None
        ctx.drop_task.release_failures = 0
        if ctx.drop_task.drop_count >= _WORLD.drop_count_target:
            # 投放阶段结束后去不去侦察由场景的 phases 决定（取代旧 stop_after_drop）。
            return TickResult(next_state=_leave_drop_phase(), fault=fault)
        # 下一投先直接飞已知的下一个候选；没有才回 SEARCH（扫描进度归零重走整条路径，
        # clusters 保留、样本跨投放累积）。
        return _next_target_or_search(ctx, fault)

    res = ctx.last_service_result
    release = {'port': ctx.drop_port}
    if res is not None and res.name == 'release_payload' and not res.success:
        ctx.drop_task.release_failures += 1
        if ctx.drop_task.release_failures > _TUNING.release_max_retries:
            ctx.last_service_result = None
            ctx.command_dispatched = False
            return finish(Fault('RELEASE_GAVE_UP', f'release_payload failed '
                                f'{ctx.drop_task.release_failures}x -> counted as attempted, moving on'))
        result = _dispatch_and_await(ctx, 'release_payload', release, finish)
        result.fault = Fault('RELEASE_FAILED', f'release_payload failed '
                                               f'({ctx.drop_task.release_failures}/{_TUNING.release_max_retries + 1}), retrying')
        return result

    # 投放执行是接口层的 release_payload（DO_SET_SERVO 定时序列，投放物理完成才返回；见
    # docs/spec/adr/0007）。DROP_PAYLOAD 态不发运动指令=飞机悬停，等投放跑完才转下一阶段。
    return _dispatch_and_await(ctx, 'release_payload', release, finish)


def _tick_navigate_to_recon_zone(ctx):
    entry = _zone_entry(_ACTIVE_SURVEY.entry_point(), _WORLD.recon_zone_center)
    arrived = _arrived(ctx.local_position_xy, entry, _TUNING.position_tolerance_m)
    if arrived or _nav_timed_out(ctx, entry):
        ctx.recon_task.phase_entered_monotonic = time.monotonic()
        _ACTIVE_SURVEY.enter(ctx)
        fault = None if arrived else Fault(
            'NAV_TIMEOUT', f'not within {_TUNING.position_tolerance_m}m of {entry} after '
                           f'{ctx.nav_timeout_s:.0f}s -> start SURVEY anyway')
        return TickResult(next_state=State.SURVEY_RECON_ZONE, fault=fault)
    return TickResult(position_target=(entry[0], entry[1], -_WORLD.cruise_alt_m),
                      speed_target_mps=_TUNING.transit_speed_mps)


def _tick_survey_recon_zone(ctx):
    # 侦察区内怎么飞、何时算覆盖完是可替换的 survey 单元（见 surveys/）；这里只把它的输出翻成
    # TickResult：覆盖完结束侦察阶段，否则发它要的位置指令（它给了限速就用它的）。
    out = _ACTIVE_SURVEY.loop(ctx)
    if out.done:
        return TickResult(next_state=State.RTL_OR_LAND)
    speeds = _zone_speeds(ctx, out.position_target[2])
    if out.speed_mps is not None:
        speeds['speed_target_mps'] = out.speed_mps
    return TickResult(position_target=out.position_target, **speeds)


def _tick_rtl_or_land(ctx):
    # 只切一次：decision_node 每拍把 last_service_result 刷成本拍结果（多数拍为 None），不记住
    # 就会隔拍重发 set_mode RTL，把飞手拨的模式抢回来（2026-10-05 任务没退出、地面解锁不了）。
    if ctx.rtl_commanded or ctx.mode == 'RTL':
        ctx.rtl_commanded = True
        return TickResult()

    def on_success():
        ctx.rtl_commanded = True
        return TickResult()
    return _dispatch_and_await(ctx, 'set_mode', {'mode': 'RTL'}, on_success)


def _tick_pilot_takeover(ctx):
    # 终态：飞手/飞控已接手，Pi 不再发任何指令、不再切模式（也不触发 RTL 守卫抢回控制）。
    return TickResult()


_DISPATCH = {
    State.INIT: _tick_init,
    State.WAIT_VEHICLE_READY: _tick_wait_vehicle_ready,
    State.WAIT_TASK_START: _tick_wait_task_start,
    State.WAIT_PILOT_ARM: _tick_wait_pilot_arm,
    State.TAKEOFF: _tick_takeoff,
    State.NAVIGATE_TO_DROP_ZONE: _tick_navigate_to_drop_zone,
    State.SEARCH_DROP_TARGETS: _tick_search_drop_targets,
    State.APPROACH_DROP_TARGET: _tick_approach_drop_target,
    State.DROP_PAYLOAD: _tick_drop_payload,
    State.NAVIGATE_TO_RECON_ZONE: _tick_navigate_to_recon_zone,
    State.SURVEY_RECON_ZONE: _tick_survey_recon_zone,
    State.RTL_OR_LAND: _tick_rtl_or_land,
    State.PILOT_TAKEOVER: _tick_pilot_takeover,
}


def _check_guards(ctx):
    """跨状态的强制打断：满足条件就返回 (强制目标状态, Fault)，命中就跳过当前
    状态本来该跑的 tick 逻辑，见 docs/spec/06-planning.md"跨状态的强制
    打断"一节——真安全响应和任务时间预算共用这同一种机制，具体阈值都
    是占位值。还没起飞、或已经在收尾阶段时，强制打断没有意义。

    飞手夺控排第一：Pi 接管后出现 Pi 不会设的模式（拨开关/飞控 failsafe）或空中被上锁，
    转 PILOT_TAKEOVER 永久停手——否则后面的电量/failsafe 守卫会再切 RTL 把控制权抢回来。"""
    if ctx.pi_in_control and ctx.current_state != State.PILOT_TAKEOVER:
        if not ctx.armed and ctx.current_state != State.RTL_OR_LAND:
            return State.PILOT_TAKEOVER, Fault('PILOT_TAKEOVER', 'disarmed in flight')
        if ctx.armed and ctx.mode is not None and ctx.mode not in _PI_MODES:
            return State.PILOT_TAKEOVER, Fault('PILOT_TAKEOVER', f'mode changed to {ctx.mode}')

    if not ctx.armed or ctx.current_state in (State.RTL_OR_LAND, State.PILOT_TAKEOVER):
        return None, None

    if ctx.battery_fraction is not None and ctx.battery_fraction < _TUNING.battery_critical_fraction:
        return State.RTL_OR_LAND, Fault('BATTERY_CRITICAL', f'battery {ctx.battery_fraction:.0%} '
                                        f'< {_TUNING.battery_critical_fraction:.0%}')
    if ctx.failsafe_active:
        return State.RTL_OR_LAND, Fault('FC_FAILSAFE', 'interface reports failsafe active')

    if ctx.current_state in _DROP_STATES and ctx.drop_task.phase_entered_monotonic is not None:
        if time.monotonic() - ctx.drop_task.phase_entered_monotonic > _TUNING.drop_phase_timeout_s:
            # 放弃投放阶段后去不去侦察同样看 phases（bench phases=[drop] 时超时直接 RTL）。
            return _leave_drop_phase(), Fault(
                'DROP_PHASE_TIMEOUT', f'{_TUNING.drop_phase_timeout_s:.0f}s in drop phase, '
                                      f'drops={ctx.drop_task.drop_count}/{_WORLD.drop_count_target}')

    if ctx.current_state in _RECON_STATES and ctx.recon_task.phase_entered_monotonic is not None:
        if time.monotonic() - ctx.recon_task.phase_entered_monotonic > _TUNING.recon_phase_timeout_s:
            return State.RTL_OR_LAND, Fault(
                'RECON_PHASE_TIMEOUT', f'{_TUNING.recon_phase_timeout_s:.0f}s in survey, '
                                       f'{_ACTIVE_SURVEY.progress()}')

    return None, None


def tick(ctx):
    """每个 tick 调一次：先查守卫链，命中就强制转移；没命中，分发给
    当前状态自己的 tick 逻辑。是否需要重置 `command_dispatched`/
    `state_entered_monotonic` 由 `decision_node.py` 在应用返回的
    `next_state` 时统一处理，这里不重复做。"""
    forced, fault = _check_guards(ctx)
    if forced is not None and forced != ctx.current_state:
        return TickResult(next_state=forced, fault=fault)
    return _DISPATCH[ctx.current_state](ctx)
