"""决策层的 ROS2 接线层：只做订阅/发布/service 调用这些 ROS2 相关的
事，不含任何任务判断逻辑——真正的状态机、每个阶段该做什么，都在
`mission_logic.py`（纯 Python，不依赖 rclpy，可以脱离节点单独测试）。
这里的职责只是每个 tick 把订阅到的最新遥测/检测组装成
`mission_logic.MissionContext`，调 `mission_logic.tick()`，再把返回的
`TickResult` 翻译成真正的 service 调用/话题发布。

对接的接口层契约见 docs/spec/04-interfaces.md，感知契约见
docs/spec/05-perception-decision-interface.md，状态机设计见
docs/spec/06-planning.md。

service 调用用 `call_async` 而不是阻塞式调用/`spin_until_future_complete`
——本节点是默认单线程 executor，阻塞式调用会卡住整个 tick 循环（同一
个节点上没有 cuadc_interface 那种独立 io 线程兜底），detail 见
docs/spec/06-planning.md 前置讨论。同一时刻只允许一个 service 调用
在途，`mission_logic.py` 的状态机本来就是线性的，不需要并发调用。
"""

import math
import time
from dataclasses import replace

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, Empty, String

from cuadc_interface_msgs.msg import FailsafeState, GpsStatus, EkfStatus, PositionTarget
from cuadc_interface_msgs.srv import Arm, SetMode, Takeoff, ReleasePayload
from cuadc_perception_msgs.msg import DetectionArray

from cuadc_decision import mission_logic
from cuadc_decision import search_strategy
from cuadc_decision.mission import config as mission_config

_INTERFACE_NS = '/cuadc_interface'

_SERVICE_SPECS = {
    'set_mode': (SetMode, f'{_INTERFACE_NS}/set_mode'),
    'arm': (Arm, f'{_INTERFACE_NS}/arm'),
    'takeoff': (Takeoff, f'{_INTERFACE_NS}/takeoff'),
    'release_payload': (ReleasePayload, f'{_INTERFACE_NS}/release_payload'),
}


def _rpy_from_quaternion(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class DecisionNode(Node):

    def __init__(self):
        super().__init__('cuadc_decision')

        # profile = sim/real 环境档（选各算法/配置层的 .sim/.real 文件）；scenario = 跑哪种
        # 场景（full_mission / drop_zone_bench / recon_zone_bench，含 initial_state 与跑哪些
        # 阶段）。两者由场景启动脚本写死（run_mission_sim.sh=sim / *_drop_zone_bench=该场景 /
        # run_mission_hardware.sh=real），见 docs/spec/mission-config-layer.md。
        self.declare_parameter('profile', 'sim')
        self.declare_parameter('scenario', 'full_mission')
        profile = self.get_parameter('profile').value
        scenario = self.get_parameter('scenario').value

        # A：世界/任务/场景（base + 场景覆盖）；B：SEARCH 算法（基线再叠场景 override）；
        # B′：FSM 本体调参；接近算法各自按 profile 载入。三层注入 mission_logic。
        world = mission_config.load(profile, scenario)
        search = replace(search_strategy.load_config(profile), **world.search_overrides)
        tuning = mission_logic.load_tuning(profile)
        mission_logic.configure_approach(profile)
        mission_logic.configure_mission(world, tuning, search)
        self._ctx = mission_logic.MissionContext(current_state=world.initial_state)
        self._pending_service_name = None
        self._pending_service_future = None
        self._yaw_rad = 0.0  # 仅诊断用：确认全程锁 yaw（末端伺服时机头不转），不进 ctx

        self._service_clients = {
            name: self.create_client(request_cls, topic)
            for name, (request_cls, topic) in _SERVICE_SPECS.items()
        }

        self._position_target_pub = self.create_publisher(
            PositionTarget, f'{_INTERFACE_NS}/cmd/position_target_local', 10)
        self._velocity_pub = self.create_publisher(
            TwistStamped, f'{_INTERFACE_NS}/cmd/velocity', 10)
        self._status_pub = self.create_publisher(String, '~/mission_state', 10)

        self.create_subscription(Bool, f'{_INTERFACE_NS}/status/connected', self._on_connected, 10)
        self.create_subscription(Bool, f'{_INTERFACE_NS}/status/armed', self._on_armed, 10)
        self.create_subscription(FailsafeState, f'{_INTERFACE_NS}/status/failsafe', self._on_failsafe, 10)
        self.create_subscription(
            PoseStamped, f'{_INTERFACE_NS}/telemetry/local_position', self._on_local_position, 10)
        self.create_subscription(
            TwistStamped, f'{_INTERFACE_NS}/telemetry/velocity', self._on_velocity, 10)
        self.create_subscription(Imu, f'{_INTERFACE_NS}/telemetry/attitude', self._on_attitude, 10)
        self.create_subscription(BatteryState, f'{_INTERFACE_NS}/telemetry/battery', self._on_battery, 10)
        self.create_subscription(GpsStatus, f'{_INTERFACE_NS}/telemetry/gps_status', self._on_gps_status, 10)
        self.create_subscription(EkfStatus, f'{_INTERFACE_NS}/telemetry/ekf_status', self._on_ekf_status, 10)
        self.create_subscription(DetectionArray, '/perception/detections', self._on_detections, 10)
        # 仿真/开发环境用来替代物理 GPIO 任务开关的触发话题（见
        # docs/spec/06-planning.md"任务触发与准备"）——真机部署时这里
        # 应该换成真实 GPIO 读取，不是 ROS2 话题；现在没有硬件目标，
        # 先用一个话题占位，方便仿真里手动/脚本触发。
        self.create_subscription(Empty, '~/sim/task_start_trigger', self._on_task_start_trigger, 10)

        self.declare_parameter('tick_rate_hz', 10.0)
        rate_hz = self.get_parameter('tick_rate_hz').value
        self._tick_budget_ms = 1000.0 / rate_hz
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

    # ---- 订阅回调：只管更新 context，不做任何判断 ----

    def _on_connected(self, msg):
        self._ctx.connected = msg.data

    def _on_armed(self, msg):
        self._ctx.armed = msg.data

    def _on_failsafe(self, msg):
        self._ctx.failsafe_active = msg.active

    def _on_local_position(self, msg):
        self._ctx.local_position_xy = (msg.pose.position.x, msg.pose.position.y)
        self._ctx.local_position_z = msg.pose.position.z

    def _on_velocity(self, msg):
        self._ctx.horizontal_speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)
        self._ctx.vertical_speed = msg.twist.linear.z

    def _on_attitude(self, msg):
        q = msg.orientation
        self._ctx.roll_rad, self._ctx.pitch_rad, self._yaw_rad = _rpy_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_battery(self, msg):
        self._ctx.battery_fraction = msg.percentage

    def _on_gps_status(self, msg):
        self._ctx.gps_fix_type = msg.fix_type

    def _on_ekf_status(self, msg):
        self._ctx.ekf_healthy = msg.healthy

    def _on_detections(self, msg):
        self._ctx.detections = list(msg.detections)

    def _on_task_start_trigger(self, msg):
        self._ctx.task_start_triggered = True

    # ---- service 调用的异步收发，见文件头说明 ----

    def _poll_pending_service_result(self):
        if self._pending_service_future is None or not self._pending_service_future.done():
            return None
        response = self._pending_service_future.result()
        result = mission_logic.ServiceResult(self._pending_service_name, bool(response.success))
        self._pending_service_future = None
        self._pending_service_name = None
        return result

    def _maybe_dispatch_service_call(self, call):
        if self._pending_service_name == call.name and self._pending_service_future is not None:
            return  # 已经在途，不重复发起
        service_type, _ = _SERVICE_SPECS[call.name]
        client = self._service_clients[call.name]
        self._pending_service_name = call.name
        self._pending_service_future = client.call_async(service_type.Request(**call.kwargs))

    # ---- 主循环 ----

    def _on_timer(self):
        self._ctx.last_service_result = self._poll_pending_service_result()

        tick_started = time.monotonic()
        result = mission_logic.tick(self._ctx)
        tick_elapsed_ms = (time.monotonic() - tick_started) * 1000.0
        # tick() 预算是一个 tick 周期（1/tick_rate_hz），只在接近/超过时才
        # 有意义地报警——纯 Python 状态机正常不会触发，是给以后决策层逻辑
        # 变重时的预警。
        if tick_elapsed_ms > self._tick_budget_ms * 0.5:
            self.get_logger().warn(
                f'[PERF] tick() took {tick_elapsed_ms:.1f}ms (budget {self._tick_budget_ms:.1f}ms)',
                throttle_duration_sec=1.0)

        if result.next_state is not None and result.next_state != self._ctx.current_state:
            self.get_logger().info(f'[STATE] {self._ctx.current_state} -> {result.next_state}')
            self._ctx.current_state = result.next_state
            self._ctx.command_dispatched = False
            self._ctx.state_entered_monotonic = time.monotonic()

        if result.position_target is not None:
            x, y, z = result.position_target
            target_msg = PositionTarget()
            target_msg.x = x
            target_msg.y = y
            target_msg.z = z
            self._position_target_pub.publish(target_msg)

        if result.velocity_target is not None:
            vx, vy, vz = result.velocity_target
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.twist.linear.x = vx  # 前（机体系），见 04-interfaces.md cmd/velocity
            twist_msg.twist.linear.y = vy  # 右
            twist_msg.twist.linear.z = vz  # 下
            twist_msg.twist.angular.z = 0.0  # yaw_rate=0：末端伺服锁当前航向、不自动转向（全程锁 yaw）
            self._velocity_pub.publish(twist_msg)

        if result.service_call is not None:
            self._maybe_dispatch_service_call(result.service_call)

        if self._ctx.current_state in (mission_logic.State.SEARCH_DROP_TARGETS,
                                        mission_logic.State.APPROACH_DROP_TARGET):
            sizes = ', '.join(f'{d.apparent_size:.2f}' for d in self._ctx.detections
                               if d.target_type == 'cylinder')
            self.get_logger().info(
                f'[DETECT] n={len(self._ctx.detections)} sizes=[{sizes}]',
                throttle_duration_sec=1.0)

        if self._ctx.current_state == mission_logic.State.SEARCH_DROP_TARGETS:
            # 诊断 SEARCH 候选聚类质量：每个聚类的均值位置 + 累计样本数,
            # 看均值有没有被斜视早样本拖偏 / 有没有把两个筒并成一簇。
            parts = []
            for c in self._ctx.drop_task.clusters:
                mx, my, _ = search_strategy._cluster_mean(c)
                parts.append(f'({mx:.1f},{my:.1f})x{len(c)}')
            self.get_logger().info(
                f'[SEARCH] clusters={len(parts)} used={len(self._ctx.drop_task.used_positions)} '
                f'[{"; ".join(parts)}]', throttle_duration_sec=1.0)

        if self._ctx.current_state == mission_logic.State.APPROACH_DROP_TARGET:
            # 诊断进近阶段抖动/收敛用，节流稍快（0.3s）才看得出振荡。yaw 确认
            # 全程锁 yaw 生效（末端应基本不变）；v 是发给飞控的机体系速度；其余
            # 直接读接近单元内部状态（诊断用，不碰控制路径）：off=跟踪到的姿态
            # 补偿水平偏移(前,右)米、miss=连续失配帧（涨=丢目标在 coast）、
            # dist2lock=到锁定目标的真实世界水平距离（地面真值，判断在靠近还是
            # 飞离）、h=离地高。
            vt = result.velocity_target
            vstr = f'({vt[0]:.2f},{vt[1]:.2f},{vt[2]:.2f})' if vt is not None else 'None'
            ast = mission_logic._ACTIVE_APPROACH.state
            off = ast.prev_off_meas
            offstr = f'({off[0]:.2f},{off[1]:.2f})' if off is not None else 'None'
            dist = None
            if ast.locked is not None and self._ctx.local_position_xy is not None:
                dist = math.hypot(self._ctx.local_position_xy[0] - ast.locked.x,
                                  self._ctx.local_position_xy[1] - ast.locked.y)
            diststr = f'{dist:.2f}' if dist is not None else 'None'
            # near_det=离锁定目标最近的实时检测世界距离。粗段停在目标上却迟迟不
            # 交接时,这个值 >refine_gate(0.4) 就说明选定的聚类位置和实测筒对不上
            # （聚类均值被斜视样本拖偏 / 并簇),即 Block B 的关联问题。
            ndist = None
            if ast.locked is not None:
                dets_xy = [(d.position.x, d.position.y) for d in self._ctx.detections
                           if d.target_type == 'cylinder']
                if dets_xy:
                    ndist = min(math.hypot(dx - ast.locked.x, dy - ast.locked.y) for dx, dy in dets_xy)
            ndstr = f'{ndist:.2f}' if ndist is not None else 'None'
            hstr = f'{-self._ctx.local_position_z:.2f}' if self._ctx.local_position_z is not None else 'None'
            self.get_logger().info(
                f'[APPROACH] phase={ast.phase} yaw={self._yaw_rad:.3f} v={vstr} '
                f'off={offstr} miss={ast.miss} dist2lock={diststr} near_det={ndstr} h={hstr} '
                f'hspeed={self._ctx.horizontal_speed}',
                throttle_duration_sec=0.3)

        self._status_pub.publish(String(data=self._ctx.current_state))


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
