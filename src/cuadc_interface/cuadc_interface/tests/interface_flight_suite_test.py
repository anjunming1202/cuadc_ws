"""接口层飞行动作合集：一次连续飞行里把树莓派→飞控的核心飞行原语走一遍，
现场用它验证「Pi 能否指挥飞控」+ 量「实机指令/传感器精度」。配合遥测地面站
（cuadc_groundstation）实时看位置/高度/速度，并开 onboard_record 录制回看——
但那两者是解耦的独立组件，本节点只驱动飞行，不启停它们。飞手 RC 全程待命。

坐标系：local_position 是起飞坐标系（原点=解锁点，x=起飞时机头方向 y=右 z=下，接口层换算，与东南西北无关，
见 docs/spec/coordinate-frames.md），
高度用 -z；水平以起飞后的 home_x/home_y 为方框/速度段中心。全程 yaw 固定=起飞朝向：位置指令走 `~/cmd/position_target_local`（绝对局部坐标，interface_node
已锁 yaw、不触发 ArduCopter 自动朝向）。

阶段（takeoff/land 恒有；中间段由 `phases` 参数选，逗号分隔，默认全开）：
  takeoff  飞手 RC 解锁 → Pi 切 GUIDED → 起飞到 takeoff_altitude_m（默认 3m；1m 地效实机发飘）
  box      定 yaw 方框（**以起飞点为中心**，±L/2 绕 前右→前左→后左→后右 再回中心）：yaw 不变
           却全向平移=定yaw全向移动，每角量单次到位误差、闭环量累计误差、并断言机头没转
  altitude 高度扫描（1→2→3→5→回起飞高度）：垂直位置控制，1m 贴近地效谨慎
  velocity 速度扫描（0.25/0.5/1/1.5/2 m/s）：**从起飞 xy 出发、沿起飞时机头方向前进→停悬
           →反向→停悬**往返；量指令→实测速度跟踪，并反复验证停发后安全悬停不续飞
  land     RTL 回 home 降落 + 自动上锁，量返航误差

最后打目标/实测/误差汇总表——sim 跑一遍给基线、real 跑一遍量精度，同格式对比。
yaw/漂移超容差只记录告警不中断。sim/real 同一节点，连接目标由 run_flight_suite_sim/real.sh 写死。

起飞条件（固定顺序，同 interface_takeoff_land_test）：放行信号 → 放行后看到一次飞手 RC 解锁
（未解锁→解锁）→ 解锁时模式是 STABILIZE → Pi 切 GUIDED 起飞。Pi 不发 arm；sim 用
`-p emulate_pilot_arm:=true` 代飞手解锁。Pi 接管后飞手拨模式/上锁 = 夺控：Pi 立即停发指令、
不再切模式、打 ABORTED 退出；只有 Pi 自己的步骤失败才兜底切 RTL。

不在 launch 里，单独手动跑：
  ros2 launch cuadc_interface interface_bringup.launch.py   # 另一个终端
  ros2 run cuadc_interface interface_flight_suite_test
  ros2 run cuadc_interface interface_flight_suite_test --ros-args -p phases:=box
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node

from cuadc_interface_msgs.msg import EkfStatus, PositionTarget
from cuadc_interface_msgs.srv import Arm, SetMode, Takeoff
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty, String

# 放行信号话题（飞行测试节点共用），同最终任务的 task_start_trigger 套路。
START_TOPIC = '/cuadc_flight_test/start'
_ARM_MODE = 'STABILIZE'                  # 飞手解锁时必须在的模式（开关不在此挡会顶掉 GUIDED）
_IN_AIR_DISARM_ALT_M = 1.0               # 上锁时离起飞点高于此 = 不是正常落地


class PilotIntervention(Exception):
    """Pi 接管后飞手/飞控改了模式或上锁：Pi 必须立即停手，不能再发任何指令。"""


def _yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class InterfaceFlightSuiteTest(Node):

    def __init__(self):
        super().__init__('interface_flight_suite_test')
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('ekf_ready_timeout_sec', 60.0)
        self.declare_parameter('ekf_settle_sec', 5.0)  # healthy 须持续此秒数再放行(抗瞬态)
        # 放行信号：默认开(现场安全)。sim 快速迭代可 -p wait_for_start:=false。
        self.declare_parameter('wait_for_start', True)
        self.declare_parameter('start_timeout_sec', 600.0)
        self.declare_parameter('pilot_arm_timeout_sec', 120.0)
        self.declare_parameter('emulate_pilot_arm', False)  # 仅 sim：节点代飞手解锁
        self.declare_parameter('phases', 'box,altitude,velocity')
        self.declare_parameter('takeoff_altitude_m', 3.0)     # 起飞/方框/速度段基准高度(1m 地效太飘)
        self.declare_parameter('takeoff_timeout_sec', 30.0)
        # 位置（方框/高度）收敛判定
        self.declare_parameter('box_leg_m', 5.0)               # 方框边长，以起飞点为中心（各向 ±L/2）
        self.declare_parameter('altitude_steps_m', [1.0, 2.0, 3.0, 5.0])  # 高度扫描，之后回起飞高度 → 1,2,3,5,回3m
        self.declare_parameter('position_tolerance_m', 0.6)
        self.declare_parameter('position_settle_sec', 2.0)
        self.declare_parameter('leg_timeout_sec', 40.0)
        self.declare_parameter('yaw_hold_tolerance_deg', 10.0)
        # 速度扫描
        self.declare_parameter('speeds_ms', [0.25, 0.5, 1.0, 1.5, 2.0])
        self.declare_parameter('vel_leg_distance_m', 3.5)  # 单向位移上限，控场地余量
        self.declare_parameter('vel_leg_max_sec', 6.0)     # 单向时长上限（低速用）
        self.declare_parameter('speed_zero_threshold_ms', 0.3)
        self.declare_parameter('speed_settle_sec', 1.5)
        self.declare_parameter('speed_zero_timeout_sec', 10.0)
        self.declare_parameter('post_stop_drift_window_sec', 3.0)
        self.declare_parameter('drift_tolerance_m', 1.0)
        # 返航降落
        self.declare_parameter('land_timeout_sec', 90.0)

        g = self.get_parameter
        self._service_timeout = g('service_timeout_sec').value
        self._ekf_ready_timeout = g('ekf_ready_timeout_sec').value
        self._ekf_settle_sec = g('ekf_settle_sec').value
        self._wait_for_start_enabled = g('wait_for_start').value
        self._start_timeout = g('start_timeout_sec').value
        self._pilot_arm_timeout = g('pilot_arm_timeout_sec').value
        self._emulate_pilot_arm = g('emulate_pilot_arm').value
        self._phases = [p.strip() for p in g('phases').value.split(',') if p.strip()]
        self._takeoff_altitude = g('takeoff_altitude_m').value
        self._takeoff_timeout = g('takeoff_timeout_sec').value
        self._box_leg = g('box_leg_m').value
        self._altitude_steps = list(g('altitude_steps_m').value)
        self._pos_tolerance = g('position_tolerance_m').value
        self._pos_settle = g('position_settle_sec').value
        self._leg_timeout = g('leg_timeout_sec').value
        self._yaw_hold_tol_deg = g('yaw_hold_tolerance_deg').value
        self._speeds = list(g('speeds_ms').value)
        self._vel_leg_distance = g('vel_leg_distance_m').value
        self._vel_leg_max_sec = g('vel_leg_max_sec').value
        self._speed_zero_threshold = g('speed_zero_threshold_ms').value
        self._speed_settle_sec = g('speed_settle_sec').value
        self._speed_zero_timeout = g('speed_zero_timeout_sec').value
        self._post_stop_drift_window = g('post_stop_drift_window_sec').value
        self._drift_tolerance = g('drift_tolerance_m').value
        self._land_timeout = g('land_timeout_sec').value

        self._arm_client = self.create_client(Arm, '/cuadc_interface/arm')
        self._set_mode_client = self.create_client(SetMode, '/cuadc_interface/set_mode')
        self._takeoff_client = self.create_client(Takeoff, '/cuadc_interface/takeoff')
        self._position_target_local_pub = self.create_publisher(
            PositionTarget, '/cuadc_interface/cmd/position_target_local', 10)
        self._velocity_pub = self.create_publisher(
            TwistStamped, '/cuadc_interface/cmd/velocity', 10)

        self._latest_armed = None
        self._latest_mode = None
        self._latest_ekf = None
        self._latest_local_pose = None
        self._latest_velocity = None
        self._latest_yaw = None
        self._start_triggered = False
        self.create_subscription(Bool, '/cuadc_interface/status/armed', self._on_armed, 10)
        self.create_subscription(String, '/cuadc_interface/status/mode', self._on_mode, 10)
        self.create_subscription(
            EkfStatus, '/cuadc_interface/telemetry/ekf_status', self._on_ekf, 10)
        self.create_subscription(
            PoseStamped, '/cuadc_interface/telemetry/local_position', self._on_local_pose, 10)
        self.create_subscription(
            TwistStamped, '/cuadc_interface/telemetry/velocity', self._on_velocity, 10)
        self.create_subscription(Imu, '/cuadc_interface/telemetry/attitude', self._on_attitude, 10)
        self.create_subscription(Empty, START_TOPIC, self._on_start, 10)

        self._results = []
        self._warnings = []  # 超容差的测量项(记录不中断),汇总时点名
        self._home_x = 0.0
        self._home_y = 0.0
        self._takeoff_yaw = 0.0
        # 接管/夺控状态：Pi 切 GUIDED 并确认后才算接管；接管后只允许 Pi 自己设过的模式。
        self._pi_in_control = False
        self._allowed_modes = set()
        self._expect_disarm = False      # Pi 发了 RTL 后的上锁是预期的
        self._disarm_alt = None
        self._intervention = None

    def run(self):
        self._wait_for_services()
        self._wait_for_ekf_ready()
        self._wait_for_start()
        self._wait_for_pilot_arm()
        self._take_control()
        try:
            self._check_takeoff()
            if 'box' in self._phases:
                self._run_box()
            if 'altitude' in self._phases:
                self._run_altitude_sweep()
            if 'velocity' in self._phases:
                self._run_velocity_sweep()
        except PilotIntervention:
            raise
        except Exception:
            # Pi 自己的步骤失败(飞手没介入)：不能直接退出让飞机在 GUIDED 里无人指挥，先尽力 RTL。
            self._failsafe_rtl()
            raise

        self._check_rtl_and_disarm()

        self._print_summary()
        phases = ",".join(self._phases)
        if self._warnings:
            self.get_logger().warn(f'interface flight-suite test COMPLETED with {len(self._warnings)} '
                                   f'out-of-tolerance item(s) (phases: {phases}):')
            for w in self._warnings:
                self.get_logger().warn(f'  - {w}')
        else:
            self.get_logger().info(f'interface flight-suite test PASSED (phases: {phases})')

    # -- 遥测回调 --

    def _on_armed(self, msg):
        was_armed = self._latest_armed
        self._latest_armed = msg.data
        if was_armed and not msg.data:
            if self._latest_local_pose is not None:
                self._disarm_alt = self._relative_altitude()
            if self._pi_in_control and not self._expect_disarm:
                self._intervention = 'vehicle DISARMED in flight (pilot RC kill or FC)'

    def _on_mode(self, msg):
        self._latest_mode = msg.data
        if self._pi_in_control and msg.data not in self._allowed_modes:
            self._intervention = f'mode changed to {msg.data or "?"} (pilot switch or FC failsafe)'

    def _on_ekf(self, msg):
        self._latest_ekf = msg

    def _on_local_pose(self, msg):
        self._latest_local_pose = msg

    def _on_velocity(self, msg):
        self._latest_velocity = msg

    def _on_attitude(self, msg):
        q = msg.orientation
        self._latest_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_start(self, msg):
        self._start_triggered = True

    # -- 辅助 --

    def _relative_altitude(self):
        return -self._latest_local_pose.pose.position.z

    def _local_xy(self):
        p = self._latest_local_pose.pose.position
        return p.x, p.y

    def _distance_xy(self, x, y):
        cx, cy = self._local_xy()
        return math.hypot(cx - x, cy - y)

    def _horizontal_speed(self):
        v = self._latest_velocity.twist.linear
        return math.hypot(v.x, v.y)

    def _warn(self, text):
        self._warnings.append(text)
        self.get_logger().warn(text)

    def _record_result(self, name, target, achieved, error):
        self._results.append((name, target, achieved, error))

    def _print_summary(self):
        self.get_logger().info('=== SUMMARY (target vs achieved vs error) ===')
        for name, target, achieved, error in self._results:
            self.get_logger().info(f'  {name}: target={target} achieved={achieved} error={error}')

    def _wait_for_condition(self, predicate, description, timeout, settle_sec=0.0):
        # 条件要连续保持 settle_sec 才算数（避免路过阈值的瞬时状态被当收敛）。
        deadline = time.monotonic() + timeout
        settled_since = None
        while time.monotonic() < deadline:
            self._spin(0.5)
            if predicate():
                if settled_since is None:
                    settled_since = time.monotonic()
                if time.monotonic() - settled_since >= settle_sec:
                    return
            else:
                settled_since = None
        raise RuntimeError(f'{description} did not happen within {timeout:.0f}s')

    def _spin(self, timeout_sec):
        # 所有等待都经这里：每处理一次回调就查一次夺控，飞手一介入立刻中断当前步骤。
        rclpy.spin_once(self, timeout_sec=timeout_sec)
        if self._intervention:
            raise PilotIntervention(self._intervention)

    def _spin_for(self, duration_sec):
        end = time.monotonic() + duration_sec
        while time.monotonic() < end:
            self._spin(0.1)

    def _call(self, client, request, name):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout)
        if self._intervention:
            raise PilotIntervention(self._intervention)
        if not future.done():
            raise RuntimeError(f'{name} call timed out')
        return future.result()

    def _wait_for_services(self):
        clients = [('set_mode', self._set_mode_client), ('takeoff', self._takeoff_client)]
        if self._emulate_pilot_arm:
            clients.append(('arm', self._arm_client))
        for name, client in clients:
            if not client.wait_for_service(timeout_sec=self._service_timeout):
                raise RuntimeError(
                    f'{name} service not available -- is cuadc_interface running and connected?')

    def _wait_for_ekf_ready(self):
        # 要求 healthy 持续 settle 秒再放行:GPS 刚锁/EKF 刚收敛时 healthy 会抖,
        # 瞬态 true 就放行会导致随后 GUIDED 被飞控拒(位置估计未稳)。
        self._wait_for_condition(
            lambda: self._latest_ekf is not None and self._latest_ekf.healthy,
            'EKF/GPS becoming healthy', self._ekf_ready_timeout,
            settle_sec=self._ekf_settle_sec)
        self.get_logger().info(f'EKF/GPS ready (healthy sustained {self._ekf_settle_sec:.0f}s)')

    def _wait_for_start(self):
        if not self._wait_for_start_enabled:
            return
        self.get_logger().warn(
            'READY — EKF ok. 从另一个 SSH 发放行(之后由飞手 RC 解锁才起飞):\n'
            f'    ros2 topic pub -t 5 -r 2 {START_TOPIC} std_msgs/msg/Empty "{{}}"')
        self._wait_for_condition(
            lambda: self._start_triggered, 'flight start trigger', self._start_timeout)
        self.get_logger().info('start trigger received')

    # -- 起飞前：等飞手解锁 → Pi 接管 --

    def _set_mode(self, mode, attempts=3, retry_delay_sec=1.5):
        if self._pi_in_control:
            self._allowed_modes.add(mode)  # 先放行再发，免得新模式的遥测先到被误判成夺控
        for i in range(1, attempts + 1):
            result = self._call(self._set_mode_client, SetMode.Request(mode=mode), 'set_mode')
            if result.success:
                self.get_logger().info(f'set_mode({mode}) confirmed')
                return
            if i < attempts:
                self.get_logger().warn(
                    f'set_mode({mode}) rejected (attempt {i}/{attempts}), retrying in '
                    f'{retry_delay_sec:.1f}s ...')
                self._spin_for(retry_delay_sec)
        raise RuntimeError(f'set_mode({mode}) returned success=False after {attempts} attempts')

    def _wait_for_pilot_arm(self):
        # 必须是放行之后的一次「未解锁→解锁」：放行前就解锁的不算，最后一个动作永远是飞手解锁。
        self._wait_for_condition(
            lambda: self._latest_armed is not None, 'armed telemetry', self._service_timeout)
        if self._latest_armed:
            self.get_logger().warn('vehicle ALREADY ARMED before start — pilot: disarm with RC, '
                                   'then re-arm when ready (Pi only takes off on a fresh arm)')
        self._wait_for_condition(
            lambda: self._latest_armed is False, 'vehicle disarmed', self._pilot_arm_timeout)
        if self._emulate_pilot_arm:
            self.get_logger().info(f'SIM: emulating pilot RC arm ({_ARM_MODE} + arm)')
            self._set_mode(_ARM_MODE)
            if not self._call(self._arm_client, Arm.Request(arm=True), 'arm').success:
                raise RuntimeError('SIM: emulated pilot arm rejected (see PreArm)')
        else:
            self.get_logger().warn(
                f'WAITING FOR PILOT ARM — mode switch on {_ARM_MODE}, then arm with RC. '
                'Pi takes off ~1s after arm.')
        self._wait_for_condition(
            lambda: self._latest_armed is True, 'pilot RC arm', self._pilot_arm_timeout)
        self._spin_for(0.3)  # 同一心跳的 mode 消息排在 armed 之后，等它到再判模式
        if self._latest_mode != _ARM_MODE:
            raise RuntimeError(
                f'armed in {self._latest_mode}, expected {_ARM_MODE} — NOT taking off. Pilot: '
                f'disarm, put the mode switch on {_ARM_MODE}, rerun')
        self.get_logger().info(f'pilot armed in {_ARM_MODE} -> Pi taking control')

    def _take_control(self):
        self._set_mode('GUIDED')
        # 等 GUIDED 的遥测真到了再算接管：之前排队的旧模式消息不能被误判成夺控。
        self._wait_for_condition(
            lambda: self._latest_mode == 'GUIDED', 'GUIDED mode telemetry', 5.0)
        if not self._latest_armed:
            raise RuntimeError('pilot disarmed before takeoff — aborting (nothing to land)')
        self._allowed_modes = {'GUIDED'}
        self._pi_in_control = True

    def _check_takeoff(self):
        self._wait_for_condition(
            lambda: self._latest_local_pose is not None and self._latest_yaw is not None,
            'local_position/attitude samples', 10.0)
        self.get_logger().info(f'calling takeoff(alt={self._takeoff_altitude}) ...')
        result = self._call(
            self._takeoff_client, Takeoff.Request(alt=self._takeoff_altitude), 'takeoff')
        if not result.success:
            raise RuntimeError('takeoff returned success=False')
        self._wait_for_condition(
            lambda: self._relative_altitude() >= self._takeoff_altitude * 0.8,
            'altitude telemetry confirming climb', self._takeoff_timeout, settle_sec=1.0)
        self._home_x, self._home_y = self._local_xy()
        self._takeoff_yaw = self._latest_yaw
        climb_err = self._relative_altitude() - self._takeoff_altitude
        self._record_result('Takeoff', f'{self._takeoff_altitude:.1f}m',
                            f'{self._relative_altitude():.2f}m', f'{climb_err:+.2f}m')
        self.get_logger().info(
            f'takeoff confirmed (alt={self._relative_altitude():.2f}m, '
            f'home=({self._home_x:.2f},{self._home_y:.2f}), yaw={math.degrees(self._takeoff_yaw):.0f}deg)')

    # -- 位置指令：起飞坐标系绝对坐标，等到收敛 --

    def _goto_local(self, fwd, right, alt, label):
        """飞到起飞坐标系 (前 fwd, 右 right)（相对方框中心 home）、高度 alt（米），yaw 不变。收敛后记录到位误差。"""
        tx, ty, tz = self._home_x + fwd, self._home_y + right, -alt
        self.get_logger().info(f'{label}: -> F={fwd:+.1f} R={right:+.1f} alt={alt:.1f} ...')
        self._position_target_local_pub.publish(PositionTarget(x=tx, y=ty, z=tz))
        # 收敛要同时满足水平到位 + 高度到位:高度扫描段 N/E 不变、水平距离一开始就≈0,
        # 只查水平会在垂直还没走完时就误判"到位"(alt_err 很大),必须把高度也纳入判据。
        self._wait_for_condition(
            lambda: self._distance_xy(tx, ty) <= self._pos_tolerance
            and abs(self._relative_altitude() - alt) <= self._pos_tolerance,
            f'{label} reaching target', self._leg_timeout, settle_sec=self._pos_settle)
        pos_err = self._distance_xy(tx, ty)
        alt_err = self._relative_altitude() - alt
        self._record_result(label, f'F={fwd:+.1f} R={right:+.1f} alt={alt:.1f}m',
                            f'pos_err={pos_err:.2f}m', f'{pos_err:.2f}m (alt_err={alt_err:+.2f}m)')
        self.get_logger().info(f'{label}: reached (pos_err={pos_err:.2f}m, alt_err={alt_err:+.2f}m)')

    def _run_box(self):
        h = self._box_leg / 2.0                 # 以起飞点为中心的方框：各向 ±half
        alt = self._takeoff_altitude
        # 起飞点在中心 → 对角飞到前右角，绕一圈(边与起飞时的前/右轴对齐)，再回中心闭环。
        corners = [
            (+h, +h, 'Box corner front-right'),
            (+h, -h, 'Box corner front-left'),
            (-h, -h, 'Box corner back-left'),
            (-h, +h, 'Box corner back-right'),
            (0.0, 0.0, 'Box back to center (close loop)'),
        ]
        for fwd, right, label in corners:
            self._goto_local(fwd, right, alt, label)
        cum_err = self._distance_xy(self._home_x, self._home_y)
        self._record_result('Box cumulative (closed loop)', 'back to takeoff point',
                            f'pos_err={cum_err:.2f}m', f'{cum_err:.2f}m')
        # 定 yaw 断言：全程只发 position_target_local，机头应没转。
        yaw_err_deg = abs(math.degrees(_angle_diff(self._latest_yaw, self._takeoff_yaw)))
        self._record_result('Box heading held (fixed yaw)', f'{math.degrees(self._takeoff_yaw):.0f}deg',
                            f'{math.degrees(self._latest_yaw):.0f}deg', f'{yaw_err_deg:.1f}deg')
        if yaw_err_deg > self._yaw_hold_tol_deg:  # 测量值:如实记录+告警,不在空中中断
            self._warn(f'heading drifted {yaw_err_deg:.1f}deg during fixed-yaw box '
                       f'(tolerance {self._yaw_hold_tol_deg:.0f}deg)')
        self.get_logger().info(
            f'box done (cumulative pos_err={cum_err:.2f}m, yaw held within {yaw_err_deg:.1f}deg)')

    def _run_altitude_sweep(self):
        # 在起飞点正上方逐级降/升，量各高度到位误差；最后爬回起飞高度。
        for alt in self._altitude_steps:
            self._goto_local(0.0, 0.0, alt, f'Altitude {alt:.1f}m')
        self._goto_local(0.0, 0.0, self._takeoff_altitude,
                        f'Altitude back to {self._takeoff_altitude:.1f}m')

    # -- 速度指令：机身前向，扫描多档速度，每档停悬 settle --

    def _velocity_leg(self, vx, label):
        """发机身前向速度 vx (m/s) 一段（时长受位移上限约束），采样实测水平速度峰值；
        然后停发、等速度降到近 0 并稳住（顺带验证停发即安全悬停）。返回实测峰值速度。"""
        dur = min(self._vel_leg_max_sec, self._vel_leg_distance / max(abs(vx), 1e-3))
        twist = TwistStamped()
        twist.twist.linear.x = vx
        t_accel_done = time.monotonic() + min(1.0, dur * 0.4)  # 前段加速不计入实测
        deadline = time.monotonic() + dur
        peak = 0.0
        while time.monotonic() < deadline:
            self._velocity_pub.publish(twist)
            self._spin(0.1)
            if time.monotonic() >= t_accel_done and self._latest_velocity is not None:
                peak = max(peak, self._horizontal_speed())
        self.get_logger().info(f'{label}: stop sending, waiting for hover settle ...')
        self._wait_for_condition(
            lambda: self._latest_velocity is not None
            and self._horizontal_speed() < self._speed_zero_threshold,
            f'{label} settling to hover', self._speed_zero_timeout, settle_sec=self._speed_settle_sec)
        return peak

    def _run_velocity_sweep(self):
        # 从起飞水平坐标开始：先回到起飞点正上方(不管上一段停在哪)，再沿起飞时机头方向往返。
        self._goto_local(0.0, 0.0, self._takeoff_altitude, 'Velocity: return to takeoff xy')
        for v in self._speeds:
            self.get_logger().info(f'velocity sweep: {v:.2f}m/s (机身前向=起飞时前方) ...')
            fwd = self._velocity_leg(v, f'Velocity {v:.2f}m/s fwd')
            back = self._velocity_leg(-v, f'Velocity {v:.2f}m/s back')
            achieved = max(fwd, back)
            self._record_result(f'Velocity {v:.2f}m/s', f'{v:.2f}m/s',
                                f'{achieved:.2f}m/s', f'{achieved - v:+.2f}m/s')
        # 收尾：已停发且悬停，再单独看一段位置漂移，确认停发后不续飞。
        sx, sy = self._local_xy()
        deadline = time.monotonic() + self._post_stop_drift_window
        max_drift = 0.0
        while time.monotonic() < deadline:
            self._spin(0.2)
            max_drift = max(max_drift, self._distance_xy(sx, sy))
        self._record_result('Velocity stop-hover drift', f'<{self._drift_tolerance:.1f}m',
                            f'max_drift={max_drift:.2f}m', f'{max_drift:.2f}m')
        if max_drift > self._drift_tolerance:  # 测量值:如实记录+告警,不在空中中断
            self._warn(f'drifted {max_drift:.2f}m after stopping '
                       f'(tolerance {self._drift_tolerance:.1f}m)')
        self.get_logger().info(f'velocity sweep done (stop-hover drift={max_drift:.2f}m)')

    # -- 返航降落 --

    def _failsafe_rtl(self):
        if self._intervention:
            return
        self.get_logger().error('flight step failed while airborne -> commanding RTL before exit')
        try:
            self._set_mode('RTL')
        except Exception as exc:  # noqa: BLE001 — 兜底本身失败只能告警,靠飞手 RC
            self.get_logger().error(f'failsafe RTL failed ({exc}) -- PILOT TAKE OVER')

    def _check_rtl_and_disarm(self):
        self.get_logger().info('calling set_mode(RTL) to return home and land ...')
        self._expect_disarm = True
        self._set_mode('RTL')
        self._wait_for_condition(
            lambda: self._latest_armed is False,
            'armed telemetry confirming auto-disarm after RTL landing', self._land_timeout)
        if self._disarm_alt is not None and self._disarm_alt > _IN_AIR_DISARM_ALT_M:
            raise RuntimeError(f'disarmed at {self._disarm_alt:.1f}m above takeoff point — '
                               'not a landing (pilot RC kill?)')
        home_err = self._distance_xy(self._home_x, self._home_y)
        self._record_result('RTL return + land', 'back to takeoff point',
                            f'home_err={home_err:.2f}m', f'{home_err:.2f}m')
        self.get_logger().info(f'RTL landed and disarmed (home return error={home_err:.2f}m)')


def main(args=None):
    rclpy.init(args=args)
    node = InterfaceFlightSuiteTest()
    exit_code = 0
    try:
        node.run()
    except PilotIntervention as exc:
        node.get_logger().warn(f'interface flight-suite test ABORTED — {exc}; '
                               'Pi stopped commanding, pilot has control')
        exit_code = 1
    except Exception as exc:  # noqa: BLE001 — 测试失败要非零退出，脚本据此判成败
        node.get_logger().error(f'interface flight-suite test FAILED: {exc}')
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
