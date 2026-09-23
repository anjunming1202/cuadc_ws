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
`mission_logic.{sim,real}.yaml`（`FsmTuning`/`load_tuning`）；SEARCH 的聚类/选目标算法
及其参数在 `search_strategy.py`（+ 同名 yaml）；投放目标选哪个在 `drop_strategy.py`；
接近算法在 `approaches/`。本文件不再持有这些为模块级常量。
"""

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import yaml

from cuadc_decision import approaches
from cuadc_decision import drop_strategy
from cuadc_decision import search_strategy
from cuadc_decision.drop_strategy import Candidate
from cuadc_decision.mission import config as mission_config


class State:
    """扁平状态机的状态名，见 docs/spec/06-planning.md。"""

    INIT = 'INIT'
    WAIT_VEHICLE_READY = 'WAIT_VEHICLE_READY'
    WAIT_TASK_START = 'WAIT_TASK_START'
    ARM_AND_TAKEOFF = 'ARM_AND_TAKEOFF'
    NAVIGATE_TO_DROP_ZONE = 'NAVIGATE_TO_DROP_ZONE'
    SEARCH_DROP_TARGETS = 'SEARCH_DROP_TARGETS'
    APPROACH_DROP_TARGET = 'APPROACH_DROP_TARGET'
    DROP_PAYLOAD = 'DROP_PAYLOAD'
    NAVIGATE_TO_RECON_ZONE = 'NAVIGATE_TO_RECON_ZONE'
    SURVEY_RECON_ZONE = 'SURVEY_RECON_ZONE'
    RTL_OR_LAND = 'RTL_OR_LAND'


@dataclass
class FsmTuning:
    """顶层任务 FSM 本体的通用调参（B′）——不属于任何可替换算法，是这个唯一的状态机
    自己的机制。字段名 = `mission_logic.<profile>.yaml` 的键。默认值只是兜底（纯单测/
    import 期不读文件），正常由 decision_node `load_tuning(profile)` 读入后注入。"""
    position_tolerance_m: float = 0.5
    altitude_tolerance_m: float = 0.5
    recon_coverage_match_m: float = 0.6       # 侦察覆盖去重：新检测离已覆盖点多近算同一个筒
    drop_phase_timeout_s: float = 90.0
    recon_phase_timeout_s: float = 90.0
    battery_critical_fraction: float = 0.20   # sensor_msgs/BatteryState.percentage 是 0~1
    gps_fix_ok_threshold: int = 3             # MAVLink GPS_FIX_TYPE：3 = 3D fix


def load_tuning(profile: str) -> FsmTuning:
    """从挨着本文件的 `mission_logic.<profile>.yaml` 读 FSM 本体调参。"""
    path = Path(__file__).parent / f'mission_logic.{profile}.yaml'
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    return FsmTuning(**data)


def _world_xy_to_ned(world_x, world_y):
    """场地坐标（cuadc_sim/field/config.py 的 x=飞行前进方向、y=侧向）
    转成接口层 telemetry/local_position 用的 NED (north, east)——两者
    不是恒等映射，X/Y 是对调的，换算依据同
    cuadc_perception/stub_perception_node.py 里的同名函数（仿真机型
    iris_with_gimbal 的 `gazeboXYZToNED` 插件参数）。2026-07-19 实机
    测试（SITL+Gazebo）发现飞机往侧边飞，就是这里之前没做这个换算、
    直接把场地坐标当 NED 坐标用导致的。
    """
    return (world_y, world_x)


def _scan_waypoints_ned(center, offsets):
    """场景配置里的区中心 + 相对 offsets（场地坐标）算成 NED 扫描航点列表。"""
    return [_world_xy_to_ned(center[0] + dx, center[1] + dy) for dx, dy in offsets]


_DROP_STATES = frozenset({State.SEARCH_DROP_TARGETS, State.APPROACH_DROP_TARGET, State.DROP_PAYLOAD})
_RECON_STATES = frozenset({State.SURVEY_RECON_ZONE})

# ---- 注入式活动配置（见文件头）。import 期用各自 dataclass 默认建一套（纯单测/import
# 不读文件）；decision_node 起来后按 profile/scenario 调 configure_*() 换成对应档。----
_WORLD = mission_config.MissionWorld()          # A：世界/任务/场景
_TUNING = FsmTuning()                            # B′：FSM 本体调参
_SEARCH = search_strategy.SearchConfig()         # B：SEARCH 算法参数
_DROP_SCAN_WAYPOINTS = _scan_waypoints_ned(_WORLD.drop_zone_center, _WORLD.drop_scan_offsets)
_RECON_SCAN_WAYPOINTS = _scan_waypoints_ned(_WORLD.recon_zone_center, _WORLD.recon_scan_offsets)

# 可替换的接近阶段策略实例（粗 PBVS -> 末端 IBVS 等，见 approaches/）。持有跨 tick 的接近
# 内部状态（锁定目标）;进入 APPROACH 时 _tick_search 调 .enter() 重置。换算法改
# approaches.ACTIVE，不动状态机。可调参数在该 approach 的 sim/real yaml。
_ACTIVE_APPROACH = approaches.create_active()


def configure_approach(profile):
    """按 sim/real profile 从 yaml 载入当前 approach 的配置,重建活动实例。
    decision_node 在 __init__ 里调一次（profile 参数由场景启动脚本给）。"""
    global _ACTIVE_APPROACH
    _ACTIVE_APPROACH = approaches.create_active(profile=profile)


def configure_mission(world, tuning, search):
    """注入世界/任务配置(A)、FSM 本体调参(B′)、SEARCH 算法配置(B)，并按 world 里的区中心/
    offsets 重算 NED 扫描航点。decision_node 在 __init__ 里调一次。"""
    global _WORLD, _TUNING, _SEARCH, _DROP_SCAN_WAYPOINTS, _RECON_SCAN_WAYPOINTS
    _WORLD, _TUNING, _SEARCH = world, tuning, search
    _DROP_SCAN_WAYPOINTS = _scan_waypoints_ned(world.drop_zone_center, world.drop_scan_offsets)
    _RECON_SCAN_WAYPOINTS = _scan_waypoints_ned(world.recon_zone_center, world.recon_scan_offsets)


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
class TickResult:
    next_state: Optional[str] = None
    position_target: Optional[Tuple[float, float, float]] = None  # (x, y, z)，同 telemetry/local_position 坐标系
    velocity_target: Optional[Tuple[float, float, float]] = None  # (前,右,下) 机体系速度 m/s，末端 IBVS 用，见 04-interfaces.md cmd/velocity
    service_call: Optional[ServiceCallRequest] = None


@dataclass
class DropTaskState:
    clusters: List[list] = field(default_factory=list)  # 每个元素是 [(x, y, size), ...] 采样列表
    used_positions: List[Tuple[float, float]] = field(default_factory=list)
    drop_count: int = 0
    selected_target: Optional[Candidate] = None
    phase_entered_monotonic: Optional[float] = None
    waypoint_index: int = 0


@dataclass
class ReconTaskState:
    covered: List[Tuple[float, float]] = field(default_factory=list)
    waypoint_index: int = 0
    phase_entered_monotonic: Optional[float] = None


@dataclass
class MissionContext:
    """一个 tick 需要知道的全部东西。`decision_node.py` 每个 tick 用
    订阅到的最新遥测/检测更新这些字段，再传给 `tick()`。"""

    current_state: str = State.INIT
    state_entered_monotonic: float = field(default_factory=time.monotonic)
    command_dispatched: bool = False
    last_service_result: Optional[ServiceResult] = None
    arm_takeoff_step: str = 'set_mode'

    connected: bool = False
    armed: bool = False
    gps_fix_type: int = 0
    ekf_healthy: bool = False
    battery_fraction: Optional[float] = None
    failsafe_active: bool = False
    local_position_xy: Optional[Tuple[float, float]] = None
    local_position_z: Optional[float] = None  # NED，向下为正
    horizontal_speed: Optional[float] = None  # 水平合速度，米/秒
    vertical_speed: Optional[float] = None  # NED vz，向下为正，米/秒
    roll_rad: Optional[float] = None
    pitch_rad: Optional[float] = None
    detections: list = field(default_factory=list)  # 元素需要有 .target_type/.position.x/.position.y/.apparent_size
    task_start_triggered: bool = False

    drop_task: DropTaskState = field(default_factory=DropTaskState)
    recon_task: ReconTaskState = field(default_factory=ReconTaskState)


# ---- 通用小工具 ----

def _arrived(current_xy, target_xy, tolerance):
    if current_xy is None:
        return False
    return math.hypot(current_xy[0] - target_xy[0], current_xy[1] - target_xy[1]) <= tolerance


def _is_near_any(x, y, points, threshold):
    return any(math.hypot(x - px, y - py) < threshold for px, py in points)


def _dispatch_and_await(ctx, service_name, kwargs, on_success):
    """一次性 service 调用的通用模式：发起一次、等结果，成功就跑
    `on_success()` 拿下一步动作，失败就重试。`set_mode`/`arm`/
    `takeoff`/`set_relay` 四处一次性调用共用这段逻辑，不各写一遍。"""
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
        return TickResult(next_state=State.ARM_AND_TAKEOFF)
    return TickResult()


def _tick_arm_and_takeoff(ctx):
    step = ctx.arm_takeoff_step

    if step == 'set_mode':
        def on_success():
            ctx.arm_takeoff_step = 'arm'
            return TickResult()
        return _dispatch_and_await(ctx, 'set_mode', {'mode': 'GUIDED'}, on_success)

    if step == 'arm':
        def on_success():
            ctx.arm_takeoff_step = 'takeoff'
            return TickResult()
        return _dispatch_and_await(ctx, 'arm', {'arm': True}, on_success)

    if step == 'takeoff':
        def on_success():
            ctx.arm_takeoff_step = 'wait_altitude'
            return TickResult()
        return _dispatch_and_await(ctx, 'takeoff', {'alt': _WORLD.cruise_alt_m}, on_success)

    # step == 'wait_altitude'：takeoff service 只确认指令被接受，不代表
    # 已经爬升到位，得自己比对遥测（04-interfaces.md 的边界约定）。
    if ctx.local_position_z is not None and -ctx.local_position_z >= _WORLD.cruise_alt_m - _TUNING.altitude_tolerance_m:
        return TickResult(next_state=State.NAVIGATE_TO_DROP_ZONE)
    return TickResult()


def _tick_navigate_to_drop_zone(ctx):
    first_waypoint = _DROP_SCAN_WAYPOINTS[0]
    if _arrived(ctx.local_position_xy, first_waypoint, _TUNING.position_tolerance_m):
        ctx.drop_task.phase_entered_monotonic = time.monotonic()
        return TickResult(next_state=State.SEARCH_DROP_TARGETS)
    return TickResult(position_target=(first_waypoint[0], first_waypoint[1], -_WORLD.cruise_alt_m))


def _tick_search_drop_targets(ctx):
    # 逐帧检测的过滤/时序聚类/候选提取/选目标门都在 search_strategy（可替换的 SEARCH
    # 算法）。这里只管：喂本帧检测、按扫描航点飞、把选中的目标交给接近阶段。只看投放区
    # 那一侧的检测由 _WORLD.drop_recon_split_y_m 分界（field-layout.md：投放区/侦察区中
    # 间隔 20m 空档，用中点粗分即可；判断的是 position.y=NED east，场地 x/y 转 NED 对调，
    # 见 `_world_xy_to_ned`）。
    search_strategy.observe(ctx, _SEARCH, _WORLD.drop_recon_split_y_m)

    # 候选还不够/还没扫完，按扫描航点继续飞，攒更多样本
    index = min(ctx.drop_task.waypoint_index, len(_DROP_SCAN_WAYPOINTS) - 1)
    waypoint = _DROP_SCAN_WAYPOINTS[index]
    scan_complete = index == len(_DROP_SCAN_WAYPOINTS) - 1 and _arrived(
        ctx.local_position_xy, waypoint, _TUNING.position_tolerance_m)
    if _arrived(ctx.local_position_xy, waypoint, _TUNING.position_tolerance_m):
        if index < len(_DROP_SCAN_WAYPOINTS) - 1:
            ctx.drop_task.waypoint_index = index + 1
            waypoint = _DROP_SCAN_WAYPOINTS[ctx.drop_task.waypoint_index]

    target = search_strategy.select_target(
        ctx, _SEARCH, scan_complete, time.monotonic(), drop_strategy.ACTIVE_STRATEGY)
    if target is not None:
        ctx.drop_task.selected_target = target
        _ACTIVE_APPROACH.enter(ctx)  # 以选定目标为锁定种子，清接近内部状态
        return TickResult(next_state=State.APPROACH_DROP_TARGET)

    return TickResult(position_target=(waypoint[0], waypoint[1], -_WORLD.cruise_alt_m))


def _tick_approach_drop_target(ctx):
    # 接近的内部实现是可替换的 approach 单元（粗 PBVS -> 末端 IBVS 等），见
    # approaches/ / cuadc_decision/README.md。这里只把它的输出翻成 TickResult：
    # 对齐并稳住（drop_ready）就转 DROP，否则发它要的运动指令（粗段位置、末端速度）。
    out = _ACTIVE_APPROACH.loop(ctx)
    if out.drop_ready:
        return TickResult(next_state=State.DROP_PAYLOAD)
    return TickResult(position_target=out.position_target, velocity_target=out.velocity_target)


def _tick_drop_payload(ctx):
    def on_success():
        target = ctx.drop_task.selected_target
        ctx.drop_task.used_positions.append((target.x, target.y))
        ctx.drop_task.drop_count += 1
        ctx.drop_task.selected_target = None
        if ctx.drop_task.drop_count >= _WORLD.drop_count_target:
            # 投放阶段结束后去不去侦察由场景的 phases 决定（取代旧 stop_after_drop）。
            if 'recon' in _WORLD.phases:
                return TickResult(next_state=State.NAVIGATE_TO_RECON_ZONE)
            return TickResult(next_state=State.RTL_OR_LAND)
        # 再次搜索前把扫描进度归零:否则 waypoint_index 停在末点，二次搜索原地
        # 不动、只吃上一轮欠采样的旧簇；归零后重走整条扫描路径给剩余筒补采样
        # （clusters 保留，样本跨投放累积）。
        ctx.drop_task.waypoint_index = 0
        return TickResult(next_state=State.SEARCH_DROP_TARGETS)

    return _dispatch_and_await(
        ctx, 'set_relay', {'channel': _WORLD.drop_relay_channel, 'state': True}, on_success)


def _tick_navigate_to_recon_zone(ctx):
    first_waypoint = _RECON_SCAN_WAYPOINTS[0]
    if _arrived(ctx.local_position_xy, first_waypoint, _TUNING.position_tolerance_m):
        ctx.recon_task.phase_entered_monotonic = time.monotonic()
        return TickResult(next_state=State.SURVEY_RECON_ZONE)
    return TickResult(position_target=(first_waypoint[0], first_waypoint[1], -_WORLD.cruise_alt_m))


def _tick_survey_recon_zone(ctx):
    for d in ctx.detections:
        if d.target_type != 'cylinder':
            continue
        if not _is_near_any(d.position.x, d.position.y, ctx.recon_task.covered, _TUNING.recon_coverage_match_m):
            ctx.recon_task.covered.append((d.position.x, d.position.y))

    if len(ctx.recon_task.covered) >= _WORLD.recon_cylinder_count:
        return TickResult(next_state=State.RTL_OR_LAND)

    index = min(ctx.recon_task.waypoint_index, len(_RECON_SCAN_WAYPOINTS) - 1)
    waypoint = _RECON_SCAN_WAYPOINTS[index]
    if _arrived(ctx.local_position_xy, waypoint, _TUNING.position_tolerance_m):
        if index < len(_RECON_SCAN_WAYPOINTS) - 1:
            ctx.recon_task.waypoint_index = index + 1
            waypoint = _RECON_SCAN_WAYPOINTS[ctx.recon_task.waypoint_index]
    return TickResult(position_target=(waypoint[0], waypoint[1], -_WORLD.cruise_alt_m))


def _tick_rtl_or_land(ctx):
    def on_success():
        # 切到 RTL 之后不需要再做什么——是否真的落地、disarm 由遥测
        # 观察即可，任务到此结束，停在这个状态直到进程退出。
        return TickResult()
    return _dispatch_and_await(ctx, 'set_mode', {'mode': 'RTL'}, on_success)


_DISPATCH = {
    State.INIT: _tick_init,
    State.WAIT_VEHICLE_READY: _tick_wait_vehicle_ready,
    State.WAIT_TASK_START: _tick_wait_task_start,
    State.ARM_AND_TAKEOFF: _tick_arm_and_takeoff,
    State.NAVIGATE_TO_DROP_ZONE: _tick_navigate_to_drop_zone,
    State.SEARCH_DROP_TARGETS: _tick_search_drop_targets,
    State.APPROACH_DROP_TARGET: _tick_approach_drop_target,
    State.DROP_PAYLOAD: _tick_drop_payload,
    State.NAVIGATE_TO_RECON_ZONE: _tick_navigate_to_recon_zone,
    State.SURVEY_RECON_ZONE: _tick_survey_recon_zone,
    State.RTL_OR_LAND: _tick_rtl_or_land,
}


def _check_guards(ctx):
    """跨状态的强制打断：满足条件就返回强制目标状态，命中就跳过当前
    状态本来该跑的 tick 逻辑，见 docs/spec/06-planning.md"跨状态的强制
    打断"一节——真安全响应和任务时间预算共用这同一种机制，具体阈值都
    是占位值。还没起飞、或已经在收尾阶段时，强制打断没有意义。"""
    if not ctx.armed or ctx.current_state == State.RTL_OR_LAND:
        return None

    if ctx.battery_fraction is not None and ctx.battery_fraction < _TUNING.battery_critical_fraction:
        return State.RTL_OR_LAND
    if ctx.failsafe_active:
        return State.RTL_OR_LAND

    if ctx.current_state in _DROP_STATES and ctx.drop_task.phase_entered_monotonic is not None:
        if time.monotonic() - ctx.drop_task.phase_entered_monotonic > _TUNING.drop_phase_timeout_s:
            # 放弃投放阶段后去不去侦察同样看 phases（bench phases=[drop] 时超时直接 RTL）。
            return State.NAVIGATE_TO_RECON_ZONE if 'recon' in _WORLD.phases else State.RTL_OR_LAND

    if ctx.current_state in _RECON_STATES and ctx.recon_task.phase_entered_monotonic is not None:
        if time.monotonic() - ctx.recon_task.phase_entered_monotonic > _TUNING.recon_phase_timeout_s:
            return State.RTL_OR_LAND

    return None


def tick(ctx):
    """每个 tick 调一次：先查守卫链，命中就强制转移；没命中，分发给
    当前状态自己的 tick 逻辑。是否需要重置 `command_dispatched`/
    `state_entered_monotonic` 由 `decision_node.py` 在应用返回的
    `next_state` 时统一处理，这里不重复做。"""
    forced = _check_guards(ctx)
    if forced is not None and forced != ctx.current_state:
        return TickResult(next_state=forced)
    return _DISPATCH[ctx.current_state](ctx)
