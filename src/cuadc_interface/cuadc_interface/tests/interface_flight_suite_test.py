"""接口层飞行动作合集：一次连续飞行里把树莓派→飞控的核心飞行原语走一遍，
现场用它验证「Pi 能否指挥飞控」+ 量「实机指令/传感器精度」。配合遥测地面站
（cuadc_groundstation）实时看位置/高度/速度，并开 onboard_record 录制回看——
但那两者是解耦的独立组件，本节点只驱动飞行，不启停它们。飞手 RC 全程待命。

坐标系：原点=起飞点，local_position 是绝对局部 NED（x=北 y=东 z=下），高度
用 -z（相对起飞点），见 docs/spec/04-interfaces.md「高度基准」。全程 yaw 固定=
起飞朝向：位置指令走 `~/cmd/position_target_local`（绝对局部坐标，interface_node
已锁 yaw、不触发 ArduCopter 自动朝向）。

阶段（takeoff/land 恒有；中间段由 `phases` 参数选，逗号分隔，默认全开）：
  takeoff  解锁 + 起飞到 takeoff_altitude_m（默认 1m 低空）
  box      定 yaw 方框（**以起飞点为中心**，±L/2 绕 NE→NW→SW→SE 再回中心）：yaw 不变
           却全向平移=定yaw全向移动，每角量单次到位误差、闭环量累计误差、并断言机头没转
  altitude 高度扫描（1→2→3→5→回 1m）：低空垂直位置控制，1m 贴近地效谨慎
  velocity 速度扫描（0.25/0.5/1/1.5/2 m/s）：**从起飞 xy 出发、沿起飞时机头方向前进→停悬
           →反向→停悬**往返；量指令→实测速度跟踪，并反复验证停发后安全悬停不续飞
  land     RTL 回 home 降落 + 自动上锁，量返航误差

最后打目标/实测/误差汇总表——sim 跑一遍给基线、real 跑一遍量精度，同格式对比。
sim/real 同一节点，连接目标由 run_flight_suite_sim/real.sh 各自写死。

放行闸门：起来后停在 arm **之前**（桨不转）、等放行，另一 SSH 发
`ros2 topic pub -1 /cuadc_flight_test/start std_msgs/msg/Empty "{}"` 才 arm+起飞。
sim 快速迭代可 `-p wait_for_start:=false` 跳过。

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
from std_msgs.msg import Bool, Empty

# arm+takeoff 前的显式放行闸门话题（两个飞行测试节点共用），同最终任务的
# task_start_trigger 套路：节点起来先停这里、桨不转，人在另一 SSH 发信号才 arm+起飞。
START_TOPIC = '/cuadc_flight_test/start'


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
        # arm+takeoff 放行闸门：默认开(现场安全)。sim 快速迭代可 -p wait_for_start:=false 直接飞。
        self.declare_parameter('wait_for_start', True)
        self.declare_parameter('start_timeout_sec', 600.0)
        self.declare_parameter('phases', 'box,altitude,velocity')
        self.declare_parameter('takeoff_altitude_m', 1.0)     # 起飞/方框/速度段都在这个低高度
        self.declare_parameter('takeoff_timeout_sec', 30.0)
        # 位置（方框/高度）收敛判定
        self.declare_parameter('box_leg_m', 5.0)               # 方框边长，以起飞点为中心（各向 ±L/2）
        self.declare_parameter('altitude_steps_m', [1.0, 2.0, 3.0, 5.0])  # 高度扫描，之后回起飞高度 → 1,2,3,5,1
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
        self._wait_for_start_enabled = g('wait_for_start').value
        self._start_timeout = g('start_timeout_sec').value
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
        self._latest_ekf = None
        self._latest_local_pose = None
        self._latest_velocity = None
        self._latest_yaw = None
        self._start_triggered = False
        self.create_subscription(Bool, '/cuadc_interface/status/armed', self._on_armed, 10)
        self.create_subscription(
            EkfStatus, '/cuadc_interface/telemetry/ekf_status', self._on_ekf, 10)
        self.create_subscription(
            PoseStamped, '/cuadc_interface/telemetry/local_position', self._on_local_pose, 10)
        self.create_subscription(
            TwistStamped, '/cuadc_interface/telemetry/velocity', self._on_velocity, 10)
        self.create_subscription(Imu, '/cuadc_interface/telemetry/attitude', self._on_attitude, 10)
        self.create_subscription(Empty, START_TOPIC, self._on_start, 10)

        self._results = []
        self._home_x = 0.0
        self._home_y = 0.0
        self._takeoff_yaw = 0.0

    def run(self):
        self._wait_for_services()
        self._wait_for_ekf_ready()
        self._wait_for_start()
        self._check_set_mode()
        self._check_arm()
        self._check_takeoff()

        if 'box' in self._phases:
            self._run_box()
        if 'altitude' in self._phases:
            self._run_altitude_sweep()
        if 'velocity' in self._phases:
            self._run_velocity_sweep()

        self._check_rtl_and_disarm()

        self._print_summary()
        self.get_logger().info('interface flight-suite test PASSED (phases: '
                               f'{",".join(self._phases)})')

    # -- 遥测回调 --

    def _on_armed(self, msg):
        self._latest_armed = msg.data

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
            rclpy.spin_once(self, timeout_sec=0.5)
            if predicate():
                if settled_since is None:
                    settled_since = time.monotonic()
                if time.monotonic() - settled_since >= settle_sec:
                    return
            else:
                settled_since = None
        raise RuntimeError(f'{description} did not happen within {timeout:.0f}s')

    def _call(self, client, request, name):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout)
        if not future.done():
            raise RuntimeError(f'{name} call timed out')
        return future.result()

    def _wait_for_services(self):
        for name, client in (
            ('arm', self._arm_client),
            ('set_mode', self._set_mode_client),
            ('takeoff', self._takeoff_client),
        ):
            if not client.wait_for_service(timeout_sec=self._service_timeout):
                raise RuntimeError(
                    f'{name} service not available -- is cuadc_interface running and connected?')

    def _wait_for_ekf_ready(self):
        self._wait_for_condition(
            lambda: self._latest_ekf is not None and self._latest_ekf.healthy,
            'EKF/GPS becoming healthy', self._ekf_ready_timeout)
        self.get_logger().info('EKF/GPS ready')

    def _wait_for_start(self):
        # 放行闸门:停在 arm 之前(桨不转),等另一 SSH 发 start。桨转前必须有人显式放行。
        if not self._wait_for_start_enabled:
            return
        self.get_logger().warn(
            'READY — EKF ok, NOT armed. arm+takeoff 已挡住,从另一个 SSH 发放行:\n'
            f'    ros2 topic pub -1 {START_TOPIC} std_msgs/msg/Empty "{{}}"')
        self._wait_for_condition(
            lambda: self._start_triggered, 'flight start trigger', self._start_timeout)
        self.get_logger().info('start trigger received -> arming + taking off')

    # -- takeoff --

    def _check_set_mode(self):
        result = self._call(self._set_mode_client, SetMode.Request(mode='GUIDED'), 'set_mode')
        if not result.success:
            raise RuntimeError('set_mode(GUIDED) returned success=False')
        self.get_logger().info('set_mode(GUIDED) confirmed')

    def _check_arm(self):
        result = self._call(self._arm_client, Arm.Request(arm=True), 'arm')
        if not result.success:
            raise RuntimeError(f'arm(True) returned success=False: {result.message}')
        self._wait_for_condition(
            lambda: self._latest_armed is True, 'armed telemetry confirming True', 10.0)
        self.get_logger().info('arm(True) confirmed')

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

    # -- 位置指令：绝对局部坐标，等到收敛 --

    def _goto_local(self, north, east, alt, label):
        """飞到相对起飞点 (north,east) 高度 alt（米），yaw 不变。收敛后记录到位误差。"""
        tx, ty, tz = self._home_x + north, self._home_y + east, -alt
        self.get_logger().info(f'{label}: -> N={north:+.1f} E={east:+.1f} alt={alt:.1f} ...')
        self._position_target_local_pub.publish(PositionTarget(x=tx, y=ty, z=tz))
        # 收敛要同时满足水平到位 + 高度到位:高度扫描段 N/E 不变、水平距离一开始就≈0,
        # 只查水平会在垂直还没走完时就误判"到位"(alt_err 很大),必须把高度也纳入判据。
        self._wait_for_condition(
            lambda: self._distance_xy(tx, ty) <= self._pos_tolerance
            and abs(self._relative_altitude() - alt) <= self._pos_tolerance,
            f'{label} reaching target', self._leg_timeout, settle_sec=self._pos_settle)
        pos_err = self._distance_xy(tx, ty)
        alt_err = self._relative_altitude() - alt
        self._record_result(label, f'N={north:+.1f} E={east:+.1f} alt={alt:.1f}m',
                            f'pos_err={pos_err:.2f}m', f'{pos_err:.2f}m (alt_err={alt_err:+.2f}m)')
        self.get_logger().info(f'{label}: reached (pos_err={pos_err:.2f}m, alt_err={alt_err:+.2f}m)')

    def _run_box(self):
        h = self._box_leg / 2.0                 # 以起飞点为中心的方框：各向 ±half
        alt = self._takeoff_altitude
        # 起飞点在中心 → 对角飞到 NE 角，绕一圈(边沿 N/E 轴对齐)，再回中心闭环。
        corners = [
            (+h, +h, 'Box corner NE'),
            (+h, -h, 'Box corner NW'),
            (-h, -h, 'Box corner SW'),
            (-h, +h, 'Box corner SE'),
            (0.0, 0.0, 'Box back to center (close loop)'),
        ]
        for north, east, label in corners:
            self._goto_local(north, east, alt, label)
        cum_err = self._distance_xy(self._home_x, self._home_y)
        self._record_result('Box cumulative (closed loop)', 'back to takeoff point',
                            f'pos_err={cum_err:.2f}m', f'{cum_err:.2f}m')
        # 定 yaw 断言：全程只发 position_target_local，机头应没转。
        yaw_err_deg = abs(math.degrees(_angle_diff(self._latest_yaw, self._takeoff_yaw)))
        self._record_result('Box heading held (fixed yaw)', f'{math.degrees(self._takeoff_yaw):.0f}deg',
                            f'{math.degrees(self._latest_yaw):.0f}deg', f'{yaw_err_deg:.1f}deg')
        if yaw_err_deg > self._yaw_hold_tol_deg:
            raise RuntimeError(
                f'heading drifted {yaw_err_deg:.1f}deg during fixed-yaw box '
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
            rclpy.spin_once(self, timeout_sec=0.1)
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
            rclpy.spin_once(self, timeout_sec=0.2)
            max_drift = max(max_drift, self._distance_xy(sx, sy))
        self._record_result('Velocity stop-hover drift', f'<{self._drift_tolerance:.1f}m',
                            f'max_drift={max_drift:.2f}m', f'{max_drift:.2f}m')
        if max_drift > self._drift_tolerance:
            raise RuntimeError(
                f'drifted {max_drift:.2f}m after stopping (tolerance {self._drift_tolerance:.1f}m)')
        self.get_logger().info(f'velocity sweep done (stop-hover drift={max_drift:.2f}m)')

    # -- 返航降落 --

    def _check_rtl_and_disarm(self):
        self.get_logger().info('calling set_mode(RTL) to return home and land ...')
        result = self._call(self._set_mode_client, SetMode.Request(mode='RTL'), 'set_mode')
        if not result.success:
            raise RuntimeError('set_mode(RTL) returned success=False')
        self._wait_for_condition(
            lambda: self._latest_armed is False,
            'armed telemetry confirming auto-disarm after RTL landing', self._land_timeout)
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
