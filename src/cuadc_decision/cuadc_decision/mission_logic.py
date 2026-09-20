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

以下几个常量是明确的占位假设，实机测试或跟硬件/场地细节确认后需要
调整，不是最终定案：`CRUISE_ALTITUDE_M`（巡航高度）、
`DROP_RELAY_CHANNEL`（投放机构用哪个 relay 通道，需要跟投放机构硬件
设计确认）、`DROP_SCAN_WAYPOINTS`/`RECON_SCAN_WAYPOINTS`（导航目标点，
基于 cuadc_sim/field/config.py 的开发用固定场地坐标算出来的，真实
比赛随机布局下这套写死的坐标不适用，需要在感知模块做实际检测/搜索
后再替换）、各种超时/容差数值。

投放目标怎么选（哪个筒先投）不在这个文件里，见 drop_strategy.py。
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from cuadc_decision import approaches
from cuadc_decision import drop_strategy
from cuadc_decision.drop_strategy import Candidate


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


# ---- 占位常量（见文件头说明） ----
_CRUISE_ALTITUDE_M = 2.0
_ALTITUDE_TOLERANCE_M = 0.5
_POSITION_TOLERANCE_M = 0.5
_SEARCH_SETTLE_SECONDS = 1.5
_CLUSTER_MATCH_THRESHOLD_M = 0.6
_MIN_CLUSTER_SAMPLES = 3
_DROP_COUNT_TARGET = 2
_DROP_CYLINDER_COUNT = 3  # 投放区规则保证固定 3 个筒，位置随机数量不随机
_DROP_RELAY_CHANNEL = 0
_MIN_DETECTION_CONFIDENCE = 0.5  # 桩节点恒发 1.0 用不上，先占住真实 CV 的过滤点

# 接近阶段的全部参数（进近半径、投放高度/容差、姿态补偿、投放门、投放机构
# 杆臂等）在 approaches/ 里该 approach 的 ApproachConfig + sim/real yaml，由可
# 替换的接近单元自己持有，见 cuadc_decision/README.md「投放接近」。


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


# 下面这些导航目标，先用场地坐标写（好对照 field/config.py 肉眼核对），
# 再统一转成 NED——不要直接改这些 _WORLD 常量成"看起来对的数字"，
# 要改就去 field/config.py 对应的圆筒/区域坐标，这里跟着重新算。
_DROP_ZONE_CENTER_WORLD = (32.5, 0.0)  # field-layout.md §8.1 C_drop
_DROP_SCAN_OFFSETS = [  # 相对中心点的 (dx, dy)，占位间距，等真实相机 FOV 定下来再调
    (-1.0, -2.0), (-1.0, 2.0),
    (0.0, 2.0), (0.0, -2.0),
    (1.0, -2.0), (1.0, 2.0),
]
_DROP_SCAN_WAYPOINTS_WORLD = [
    (_DROP_ZONE_CENTER_WORLD[0] + dx, _DROP_ZONE_CENTER_WORLD[1] + dy)
    for dx, dy in _DROP_SCAN_OFFSETS
]
_DROP_SCAN_WAYPOINTS = [_world_xy_to_ned(wx, wy) for wx, wy in _DROP_SCAN_WAYPOINTS_WORLD]

_RECON_SCAN_WAYPOINTS_WORLD = [
    (56.0, -2.0), (56.0, 2.0),
    (57.5, 2.0), (57.5, -2.0),
    (59.0, -2.0), (59.0, 2.0),
]
_RECON_SCAN_WAYPOINTS = [_world_xy_to_ned(wx, wy) for wx, wy in _RECON_SCAN_WAYPOINTS_WORLD]
_RECON_CYLINDER_COUNT = 5
_BATTERY_CRITICAL_FRACTION = 0.20  # sensor_msgs/BatteryState.percentage 是 0~1
_GPS_FIX_OK_THRESHOLD = 3  # MAVLink GPS_FIX_TYPE：3 = 3D fix
_DROP_PHASE_TIMEOUT_S = 90.0
_RECON_PHASE_TIMEOUT_S = 90.0

_DROP_STATES = frozenset({State.SEARCH_DROP_TARGETS, State.APPROACH_DROP_TARGET, State.DROP_PAYLOAD})
_RECON_STATES = frozenset({State.SURVEY_RECON_ZONE})

# 可替换的接近阶段策略实例（粗 PBVS -> 末端 IBVS 等，见 approaches/）。持有跨
# tick 的接近内部状态（锁定目标）;进入 APPROACH 时 _tick_search 调 .enter() 重置。
# 换算法改 approaches.ACTIVE，不动状态机。可调参数在该 approach 的 sim/real yaml。
# 这里用 dataclass 默认建一个（纯单测/import 期不读文件）;decision_node 起来后按
# profile 调 configure_approach() 换成对应档,见下。
_ACTIVE_APPROACH = approaches.create_active()


def configure_approach(profile):
    """按 sim/real profile 从 yaml 载入当前 approach 的配置,重建活动实例。
    decision_node 在 __init__ 里调一次（profile 参数由场景启动脚本给）。"""
    global _ACTIVE_APPROACH
    _ACTIVE_APPROACH = approaches.create_active(profile=profile)


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
    stop_after_drop: bool = False  # 硬件单点测试用：投完两次直接 RTL，不去侦察区

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


# ---- 投放目标选哪个：见 drop_strategy.py，这里只负责调用 ----

def _update_clusters(clusters, points, match_threshold):
    """按最近邻把本帧检测归并进已有聚类（同一个物理目标的历史采样），
    没有匹配上的新建一个聚类——感知无状态、不维护 track_id，这个归并
    是决策层自己做的（见 05-perception-decision-interface.md）。"""
    for x, y, size in points:
        best = None
        best_dist = match_threshold
        for cluster in clusters:
            cx, cy, _ = _cluster_mean(cluster)
            dist = math.hypot(x - cx, y - cy)
            if dist < best_dist:
                best_dist = dist
                best = cluster
        if best is None:
            clusters.append([(x, y, size)])
        else:
            best.append((x, y, size))


def _cluster_mean(cluster):
    n = len(cluster)
    return (
        sum(p[0] for p in cluster) / n,
        sum(p[1] for p in cluster) / n,
        sum(p[2] for p in cluster) / n,
    )


# ---- 各状态的 tick 逻辑 ----

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
    if ctx.ekf_healthy and ctx.gps_fix_type >= _GPS_FIX_OK_THRESHOLD:
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
        return _dispatch_and_await(ctx, 'takeoff', {'alt': _CRUISE_ALTITUDE_M}, on_success)

    # step == 'wait_altitude'：takeoff service 只确认指令被接受，不代表
    # 已经爬升到位，得自己比对遥测（04-interfaces.md 的边界约定）。
    if ctx.local_position_z is not None and -ctx.local_position_z >= _CRUISE_ALTITUDE_M - _ALTITUDE_TOLERANCE_M:
        return TickResult(next_state=State.NAVIGATE_TO_DROP_ZONE)
    return TickResult()


def _tick_navigate_to_drop_zone(ctx):
    first_waypoint = _DROP_SCAN_WAYPOINTS[0]
    if _arrived(ctx.local_position_xy, first_waypoint, _POSITION_TOLERANCE_M):
        ctx.drop_task.phase_entered_monotonic = time.monotonic()
        return TickResult(next_state=State.SEARCH_DROP_TARGETS)
    return TickResult(position_target=(first_waypoint[0], first_waypoint[1], -_CRUISE_ALTITUDE_M))


def _tick_search_drop_targets(ctx):
    # 只看投放区那一侧的检测（field-layout.md：投放区场地 x in [30,35]，
    # 侦察区场地 x in [55,60]，中间隔了 20m 空档，用中点粗略分开即可，
    # Detection 本身不带区域信息）。注意判断的是 position.y（NED east），
    # 不是 position.x——场地坐标的 x/y 转成 NED 时是对调的，见
    # `_world_xy_to_ned` 的说明。
    points = [
        (d.position.x, d.position.y, d.apparent_size)
        for d in ctx.detections
        if d.target_type == 'cylinder' and d.confidence >= _MIN_DETECTION_CONFIDENCE and d.position.y < 45.0
    ]
    _update_clusters(ctx.drop_task.clusters, points, _CLUSTER_MATCH_THRESHOLD_M)

    # 候选还不够/还没扫完，按扫描航点继续飞，攒更多样本
    index = min(ctx.drop_task.waypoint_index, len(_DROP_SCAN_WAYPOINTS) - 1)
    waypoint = _DROP_SCAN_WAYPOINTS[index]
    scan_complete = index == len(_DROP_SCAN_WAYPOINTS) - 1 and _arrived(
        ctx.local_position_xy, waypoint, _POSITION_TOLERANCE_M)
    if _arrived(ctx.local_position_xy, waypoint, _POSITION_TOLERANCE_M):
        if index < len(_DROP_SCAN_WAYPOINTS) - 1:
            ctx.drop_task.waypoint_index = index + 1
            waypoint = _DROP_SCAN_WAYPOINTS[ctx.drop_task.waypoint_index]

    if time.monotonic() - ctx.state_entered_monotonic >= _SEARCH_SETTLE_SECONDS:
        candidates = [
            Candidate(*_cluster_mean(c))
            for c in ctx.drop_task.clusters
            if len(c) >= _MIN_CLUSTER_SAMPLES
        ]
        # 3 个筒都找齐了，或者已经飞完整条扫描路径都没能找齐——两种情况
        # 才允许选目标，不能"随便凑够一个候选就选"：不然可能锁死在第一眼
        # 看到的筒上，从没飞去比较过其它候选（2026-09-11 SITL 实测踩过）。
        if len(candidates) >= _DROP_CYLINDER_COUNT or scan_complete:
            target = drop_strategy.ACTIVE_STRATEGY(candidates, ctx.drop_task.used_positions, ctx.drop_task.drop_count)
            if target is not None:
                ctx.drop_task.selected_target = target
                _ACTIVE_APPROACH.enter(ctx)  # 以选定目标为锁定种子，清接近内部状态
                return TickResult(next_state=State.APPROACH_DROP_TARGET)

    return TickResult(position_target=(waypoint[0], waypoint[1], -_CRUISE_ALTITUDE_M))


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
        if ctx.drop_task.drop_count >= _DROP_COUNT_TARGET:
            if ctx.stop_after_drop:
                return TickResult(next_state=State.RTL_OR_LAND)
            return TickResult(next_state=State.NAVIGATE_TO_RECON_ZONE)
        return TickResult(next_state=State.SEARCH_DROP_TARGETS)

    return _dispatch_and_await(
        ctx, 'set_relay', {'channel': _DROP_RELAY_CHANNEL, 'state': True}, on_success)


def _tick_navigate_to_recon_zone(ctx):
    first_waypoint = _RECON_SCAN_WAYPOINTS[0]
    if _arrived(ctx.local_position_xy, first_waypoint, _POSITION_TOLERANCE_M):
        ctx.recon_task.phase_entered_monotonic = time.monotonic()
        return TickResult(next_state=State.SURVEY_RECON_ZONE)
    return TickResult(position_target=(first_waypoint[0], first_waypoint[1], -_CRUISE_ALTITUDE_M))


def _tick_survey_recon_zone(ctx):
    for d in ctx.detections:
        if d.target_type != 'cylinder':
            continue
        if not _is_near_any(d.position.x, d.position.y, ctx.recon_task.covered, _CLUSTER_MATCH_THRESHOLD_M):
            ctx.recon_task.covered.append((d.position.x, d.position.y))

    if len(ctx.recon_task.covered) >= _RECON_CYLINDER_COUNT:
        return TickResult(next_state=State.RTL_OR_LAND)

    index = min(ctx.recon_task.waypoint_index, len(_RECON_SCAN_WAYPOINTS) - 1)
    waypoint = _RECON_SCAN_WAYPOINTS[index]
    if _arrived(ctx.local_position_xy, waypoint, _POSITION_TOLERANCE_M):
        if index < len(_RECON_SCAN_WAYPOINTS) - 1:
            ctx.recon_task.waypoint_index = index + 1
            waypoint = _RECON_SCAN_WAYPOINTS[ctx.recon_task.waypoint_index]
    return TickResult(position_target=(waypoint[0], waypoint[1], -_CRUISE_ALTITUDE_M))


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

    if ctx.battery_fraction is not None and ctx.battery_fraction < _BATTERY_CRITICAL_FRACTION:
        return State.RTL_OR_LAND
    if ctx.failsafe_active:
        return State.RTL_OR_LAND

    if ctx.current_state in _DROP_STATES and ctx.drop_task.phase_entered_monotonic is not None:
        if time.monotonic() - ctx.drop_task.phase_entered_monotonic > _DROP_PHASE_TIMEOUT_S:
            return State.NAVIGATE_TO_RECON_ZONE

    if ctx.current_state in _RECON_STATES and ctx.recon_task.phase_entered_monotonic is not None:
        if time.monotonic() - ctx.recon_task.phase_entered_monotonic > _RECON_PHASE_TIMEOUT_S:
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
