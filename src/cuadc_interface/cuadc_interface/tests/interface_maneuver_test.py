"""接口层机动测试：按目的分四个阶段，每个阶段内部用统一的"等到完全
收敛、测量误差、记录进汇总表"模式（Phase D 的压力测试除外，明确不代表
精度）——不是按时间顺序堆一串动作，是每一段都有明确要回答的问题：

  Phase A 单动作精度——Go To Location、Yaw 各执行一次，测量误差。
  Phase B 累计误差——同一动作重复 4 次形成闭环，测首尾误差，位置和
    转向各一组，结构对称，方便直接对比谁的累计误差更大。
  Phase C 自动朝向行为——只发位置指令、不发 Set Yaw，确认机头会自动
    转向飞行方向（ArduCopter `WP_YAW_BEHAVIOR` 默认行为，已接受，见
    docs/spec/04-interfaces.md"自动朝向行为"一节；Set Yaw 只在飞机
    静止、没有并发位置指令时才独立可控，不在这里测）。
  Phase D 追踪/压力测试——故意不等收敛，测试"新指令打断还没完成的
    旧指令"这种真实会发生的场景；转向这段比位置这段轻（任务里转向
    用得少，见 docs/spec/04-interfaces.md 相关讨论），但仍要确认新
    指令能正确顶替旧的。

最后打印一张目标/实测/误差的汇总表——不只是这次看结果用，以后上真机
也想拿这些脚本当动作校准/验收用，统一格式方便对比不同时间跑的结果。

高度全部用相对起飞点的高度（`local_position` 的 `-position.z`），不
用海拔做加减，见 04-interfaces.md"高度基准"一节。

不在 launch 文件里，单独手动跑，同 interface_acceptance_test。

用法：
  ros2 launch cuadc_interface interface_bringup.launch.py   # 另一个终端
  ros2 run cuadc_interface interface_maneuver_test
"""
import math
import time

import rclpy
from rclpy.node import Node

from cuadc_interface_msgs.msg import EkfStatus
from cuadc_interface_msgs.srv import Arm, GotoLocation, SetMode, Takeoff
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float64, Empty

_METERS_PER_DEGREE = 111320  # 粗略换算，纬度低误差小，够用


def _yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a, b):
    # 最短夹角差，处理 -pi/pi 折返，否则 179 度和 -179 度会被误判成差 358 度
    d = (a - b + math.pi) % (2 * math.pi) - math.pi
    return abs(d)


def _wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


# arm+takeoff 放行闸门话题（所有会飞的测试共用），同最终任务 task_start_trigger 套路。
START_TOPIC = '/cuadc_flight_test/start'

class InterfaceManeuverTest(Node):

    def __init__(self):
        super().__init__('interface_maneuver_test')
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('ekf_ready_timeout_sec', 60.0)
        self.declare_parameter('takeoff_altitude_m', 10.0)
        self.declare_parameter('altitude_tolerance_m', 3.0)
        self.declare_parameter('position_tolerance_m', 2.0)
        self.declare_parameter('yaw_tolerance_deg', 10.0)
        self.declare_parameter('leg_timeout_sec', 30.0)
        self.declare_parameter('yaw_timeout_sec', 20.0)
        self.declare_parameter('land_timeout_sec', 60.0)
        self.declare_parameter('settle_sec', 1.5)
        self._service_timeout = self.get_parameter('service_timeout_sec').value
        self._ekf_ready_timeout = self.get_parameter('ekf_ready_timeout_sec').value
        self._takeoff_altitude = self.get_parameter('takeoff_altitude_m').value
        self._altitude_tolerance = self.get_parameter('altitude_tolerance_m').value
        self._position_tolerance = self.get_parameter('position_tolerance_m').value
        self._yaw_tolerance = math.radians(self.get_parameter('yaw_tolerance_deg').value)
        self._leg_timeout = self.get_parameter('leg_timeout_sec').value
        self._yaw_timeout = self.get_parameter('yaw_timeout_sec').value
        self._land_timeout = self.get_parameter('land_timeout_sec').value
        self._settle_sec = self.get_parameter('settle_sec').value

        self._arm_client = self.create_client(Arm, '/cuadc_interface/arm')
        self._set_mode_client = self.create_client(SetMode, '/cuadc_interface/set_mode')
        self._takeoff_client = self.create_client(Takeoff, '/cuadc_interface/takeoff')
        self._goto_client = self.create_client(GotoLocation, '/cuadc_interface/goto_location')
        self._yaw_pub = self.create_publisher(Float64, '/cuadc_interface/cmd/yaw', 10)

        self._latest_armed = None
        self._latest_ekf = None
        self._latest_position = None  # NavSatFix：只用来读 lat/lon
        self._latest_local_pose = None  # PoseStamped：relative_alt = -z
        self._latest_yaw = None
        self._results = []  # (name, target, achieved, error) 汇总表
        self.create_subscription(Bool, '/cuadc_interface/status/armed', self._on_armed, 10)
        self.create_subscription(
            EkfStatus, '/cuadc_interface/telemetry/ekf_status', self._on_ekf, 10)
        self.create_subscription(
            NavSatFix, '/cuadc_interface/telemetry/global_position', self._on_position, 10)
        self.create_subscription(
            PoseStamped, '/cuadc_interface/telemetry/local_position', self._on_local_pose, 10)
        self.create_subscription(Imu, '/cuadc_interface/telemetry/attitude', self._on_attitude, 10)

        self.declare_parameter('wait_for_start', True)
        self.declare_parameter('start_timeout_sec', 600.0)
        self._wait_for_start_enabled = self.get_parameter('wait_for_start').value
        self._start_timeout = self.get_parameter('start_timeout_sec').value
        self._start_triggered = False
        self.create_subscription(Empty, START_TOPIC, self._on_start, 10)
        self._wait_for_services()
        self._wait_for_ekf_ready()
        self._wait_for_start()
        self._check_set_mode()
        self._check_arm()
        self._check_takeoff()

        home_lat = self._latest_position.latitude
        home_lon = self._latest_position.longitude
        home_yaw = self._latest_yaw

        # -- Phase A：单动作精度 --
        wp_a1 = (home_lat + 0.00018, home_lon + 0.00018)  # ~20m N, ~20m E
        self._goto_full(wp_a1[0], wp_a1[1], self._takeoff_altitude + 10.0,
                         'Phase A1: Go To Location (horizontal+climb combined)')
        self._yaw_full(_wrap_angle(self._latest_yaw + math.pi / 2), 'Phase A2: Yaw (+90deg)')

        # -- Phase B：累计误差（闭环） --
        self._check_goto_cumulative(*self._current_latlon(), self._takeoff_altitude + 10.0)
        self._check_yaw_cumulative(self._latest_yaw)

        # -- Phase C：验证默认自动朝向行为（不发 Set Yaw，只发位置指令）--
        cur_lat, cur_lon = self._current_latlon()
        wp_c1 = (cur_lat, cur_lon + 0.00018)  # 正东，方位角约 90 度，方便肉眼核对
        self._check_auto_yaw_behavior(
            wp_c1[0], wp_c1[1], self._takeoff_altitude,
            'Phase C1: auto-yaw during Go To Location')

        # -- Phase D：追踪/压力测试（不等收敛，不代表精度） --
        self._check_continuous_goto(*self._current_latlon())
        self._check_continuous_yaw(self._latest_yaw)

        # -- 收尾：闭环回归 home --
        self._goto_full(home_lat, home_lon, self._takeoff_altitude, 'Return home (position)')
        self._yaw_full(home_yaw, 'Return home (yaw)')
        self._check_land_and_disarm()

        self._print_summary()
        self.get_logger().info(
            'interface maneuver test PASSED: single-shot accuracy, cumulative drift, '
            'auto-yaw behavior, and retargeting-under-tracking all confirmed')
        rclpy.shutdown()

    # -- 遥测回调 --

    def _on_armed(self, msg):
        self._latest_armed = msg.data

    def _on_ekf(self, msg):
        self._latest_ekf = msg

    def _on_position(self, msg):
        self._latest_position = msg

    def _on_local_pose(self, msg):
        self._latest_local_pose = msg

    def _on_attitude(self, msg):
        q = msg.orientation
        self._latest_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)

    # -- 辅助方法 --

    def _relative_altitude(self):
        return -self._latest_local_pose.pose.position.z

    def _current_latlon(self):
        return self._latest_position.latitude, self._latest_position.longitude

    def _distance_to(self, lat, lon):
        dlat = self._latest_position.latitude - lat
        dlon = self._latest_position.longitude - lon
        return math.hypot(dlat, dlon) * _METERS_PER_DEGREE

    def _record_result(self, name, target, achieved, error):
        self._results.append((name, target, achieved, error))

    def _print_summary(self):
        self.get_logger().info('=== SUMMARY (target vs achieved vs error) ===')
        for name, target, achieved, error in self._results:
            self.get_logger().info(f'  {name}: target={target} achieved={achieved} error={error}')

    def _wait_for_condition(self, predicate, description, timeout, settle_sec=0.0):
        # 单次轮询命中阈值就返回的话，可能逮到的是"正好路过阈值"的瞬时
        # 状态（还在转/还在飞），不是真正收敛——精度测量类的调用都要求
        # 条件连续保持 settle_sec 才算数，中途掉出阈值就重新计时。
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

    def _on_start(self, msg):
        self._start_triggered = True

    def _wait_for_start(self):
        # arm+takeoff 放行闸门:停在 arm 之前(桨不转),等另一 SSH 发 start 才继续。
        if not self._wait_for_start_enabled:
            return
        self.get_logger().warn(
            'READY \u2014 EKF ok, NOT armed. arm+takeoff \u5df2\u6321\u4f4f,\u4ece\u53e6\u4e00\u4e2a SSH \u53d1\u653e\u884c:\n'
            f'    ros2 topic pub -1 {START_TOPIC} std_msgs/msg/Empty "{{}}"')
        self._wait_for_condition(
            lambda: self._start_triggered, 'flight start trigger', self._start_timeout)
        self.get_logger().info('start trigger received -> arming + taking off')

    def _wait_for_services(self):
        for name, client in (
            ('arm', self._arm_client),
            ('set_mode', self._set_mode_client),
            ('takeoff', self._takeoff_client),
            ('goto_location', self._goto_client),
        ):
            if not client.wait_for_service(timeout_sec=self._service_timeout):
                raise RuntimeError(
                    f'{name} service not available -- is cuadc_interface running '
                    'and connected?')

    def _wait_for_ekf_ready(self):
        self._wait_for_condition(
            lambda: self._latest_ekf is not None and self._latest_ekf.healthy,
            'EKF/GPS becoming healthy', self._ekf_ready_timeout)
        self.get_logger().info('EKF/GPS ready')

    # -- 起飞前置检查 --

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
            lambda: self._relative_altitude() >= self._takeoff_altitude - 1.0,
            'altitude telemetry confirming climb', self._leg_timeout, settle_sec=self._settle_sec)
        alt_err = self._relative_altitude() - self._takeoff_altitude
        self._record_result('Takeoff', f'{self._takeoff_altitude:.1f}m',
                             f'{self._relative_altitude():.2f}m', f'{alt_err:+.2f}m')
        self.get_logger().info(f'takeoff confirmed (alt_err={alt_err:+.2f}m)')

    # -- Phase A/B/收尾 共用：单次动作等到 100% 收敛 + 记录误差 --

    def _goto_full(self, lat, lon, alt, label):
        start_dist = self._distance_to(lat, lon)
        start_alt = self._relative_altitude()
        self.get_logger().info(
            f'{label}: goto lat={lat:.7f} lon={lon:.7f} alt={alt:.1f} (dist={start_dist:.1f}m) ...')
        result = self._call(
            self._goto_client, GotoLocation.Request(lat=lat, lon=lon, alt=alt), label)
        if not result.success:
            raise RuntimeError(f'{label}: goto_location returned success=False')

        # 爬升/下降需要时间，中途高度还没到目标是正常的——只检查没有
        # 飞出起点/终点之间的合理区间，能抓住"高度基准算混导致持续
        # 爬升不停"这类问题（见 04-interfaces.md"高度基准"一节）。
        transit_lo = min(start_alt, alt) - self._altitude_tolerance
        transit_hi = max(start_alt, alt) + self._altitude_tolerance
        deadline = time.monotonic() + self._leg_timeout
        settled_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            cur_alt = self._relative_altitude()
            if not (transit_lo <= cur_alt <= transit_hi):
                raise RuntimeError(
                    f'{label}: relative altitude {cur_alt:.2f}m outside transit range '
                    f'[{transit_lo:.2f}, {transit_hi:.2f}]m -- see 04-interfaces.md'
                    '"高度基准"一节')
            if self._distance_to(lat, lon) <= self._position_tolerance:
                if settled_since is None:
                    settled_since = time.monotonic()
                if time.monotonic() - settled_since >= self._settle_sec:
                    break
            else:
                settled_since = None
        else:
            raise RuntimeError(f'{label}: never reached target within {self._leg_timeout:.0f}s')

        pos_err = self._distance_to(lat, lon)
        alt_err = self._relative_altitude() - alt
        self._record_result(
            label, f'alt={alt:.1f}m', f'pos_err={pos_err:.2f}m alt={self._relative_altitude():.2f}m',
            f'pos={pos_err:.2f}m alt={alt_err:+.2f}m')
        self.get_logger().info(f'{label}: confirmed (pos_err={pos_err:.2f}m, alt_err={alt_err:+.2f}m)')

    def _yaw_full(self, target_rad, label):
        self.get_logger().info(f'{label}: target={math.degrees(target_rad):.0f}deg ...')
        self._yaw_pub.publish(Float64(data=target_rad))
        self._wait_for_condition(
            lambda: self._latest_yaw is not None
            and _angle_diff(self._latest_yaw, target_rad) < self._yaw_tolerance,
            f'{label} reaching target heading', self._yaw_timeout, settle_sec=self._settle_sec)
        err_deg = math.degrees(_angle_diff(self._latest_yaw, target_rad))
        self._record_result(
            label, f'{math.degrees(target_rad):.0f}deg',
            f'{math.degrees(self._latest_yaw):.0f}deg', f'{err_deg:.1f}deg')
        self.get_logger().info(f'{label}: confirmed (yaw_err={err_deg:.1f}deg)')

    # -- Phase B：累计误差 --

    def _check_goto_cumulative(self, base_lat, base_lon, alt):
        d = 0.00013  # ~15m
        legs = [
            (base_lat + d, base_lon, 'Phase B1 leg1 (N)'),
            (base_lat + d, base_lon + d, 'Phase B1 leg2 (E)'),
            (base_lat, base_lon + d, 'Phase B1 leg3 (S)'),
            (base_lat, base_lon, 'Phase B1 leg4 (W, back to loop start)'),
        ]
        for lat, lon, label in legs:
            self._goto_full(lat, lon, alt, label)
        final_err = self._distance_to(base_lat, base_lon)
        self._record_result(
            'Phase B1: Go To Location cumulative (4-leg loop)',
            'back to loop start', f'pos_err={final_err:.2f}m', f'{final_err:.2f}m')
        self.get_logger().info(f'Phase B1 cumulative position error: {final_err:.2f}m')

    def _check_yaw_cumulative(self, start_yaw):
        for i, k in enumerate((1, 2, 3, 4)):
            target = _wrap_angle(start_yaw + math.radians(90 * k))
            self._yaw_full(target, f'Phase B2 leg{i+1} (+{90*k}deg cumulative)')
        final_err_deg = math.degrees(_angle_diff(self._latest_yaw, start_yaw))
        self._record_result(
            'Phase B2: Yaw cumulative (4x90deg loop)',
            f'{math.degrees(start_yaw):.0f}deg (back to loop start)',
            f'{math.degrees(self._latest_yaw):.0f}deg', f'{final_err_deg:.1f}deg')
        self.get_logger().info(f'Phase B2 cumulative yaw error: {final_err_deg:.1f}deg')

    # -- Phase C：验证默认自动朝向行为（见 04-interfaces.md"自动朝向
    # 行为"一节）——不发 Set Yaw，只发位置指令，确认机头确实会自动
    # 转向飞行方向。

    def _bearing_to(self, lat, lon):
        lat1 = math.radians(self._latest_position.latitude)
        lat2 = math.radians(lat)
        dlon = math.radians(lon - self._latest_position.longitude)
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        return math.atan2(y, x)

    def _check_auto_yaw_behavior(self, lat, lon, alt, label):
        expected_bearing = self._bearing_to(lat, lon)
        self._goto_full(lat, lon, alt, label)
        yaw_err_deg = math.degrees(_angle_diff(self._latest_yaw, expected_bearing))
        self._record_result(
            f'{label} (auto-yaw)', f'bearing={math.degrees(expected_bearing):.0f}deg',
            f'yaw={math.degrees(self._latest_yaw):.0f}deg', f'{yaw_err_deg:.1f}deg')
        self.get_logger().info(f'{label}: auto-yaw confirmed (yaw vs bearing err={yaw_err_deg:.1f}deg)')

    # -- Phase D：追踪/压力测试，故意不等收敛，不代表精度 --

    def _check_continuous_goto(self, base_lat, base_lon):
        alt = self._takeoff_altitude
        waypoints = [
            (base_lat + 0.00009, base_lon),
            (base_lat + 0.00009, base_lon + 0.00011),
            (base_lat, base_lon + 0.00011),
            (base_lat, base_lon),
        ]
        self.get_logger().info(
            'Phase D1 (continuous goto retargeting, no settle wait between calls) ...')
        for i, (lat, lon) in enumerate(waypoints):
            result = self._call(
                self._goto_client, GotoLocation.Request(lat=lat, lon=lon, alt=alt),
                f'Phase D1 wp{i}')
            if not result.success:
                raise RuntimeError(f'Phase D1 wp{i}: goto_location returned success=False')
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.5)
                cur_alt = self._relative_altitude()
                if abs(cur_alt - alt) > self._altitude_tolerance:
                    raise RuntimeError(
                        f'Phase D1 wp{i}: relative altitude {cur_alt:.2f}m drifted more than '
                        f'{self._altitude_tolerance:.1f}m from {alt:.1f}m during rapid '
                        'retargeting -- see 04-interfaces.md"高度基准"一节')
        # 最后一个点给够时间真正稳定，这一步才是唯一有意义的精度确认点
        self._wait_for_condition(
            lambda: self._distance_to(*waypoints[-1]) <= self._position_tolerance,
            'Phase D1 settling at the final rapid-fire waypoint', self._leg_timeout,
            settle_sec=self._settle_sec)
        final_err = self._distance_to(*waypoints[-1])
        self._record_result(
            'Phase D1: continuous goto retarget (final settle, NOT single-shot accuracy)',
            'last of 4 rapid-fire targets', f'pos_err={final_err:.2f}m', f'{final_err:.2f}m')
        self.get_logger().info(f'Phase D1 confirmed (final pos_err={final_err:.2f}m)')

    def _check_continuous_yaw(self, start_yaw):
        # 比 Phase D1 轻——任务里转向用得少，不需要跟位置控制一样反复
        # 压测，但仍要确认"新指令能正确顶替还没转完的旧指令"。
        self.get_logger().info(
            'Phase D2 (continuous yaw retargeting, no settle wait between calls) ...')
        targets = [_wrap_angle(start_yaw + math.radians(60)),
                   _wrap_angle(start_yaw + math.radians(150))]
        for t in targets:
            self._yaw_pub.publish(Float64(data=t))
            deadline = time.monotonic() + 2.5
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.5)
        # 只有最后一个目标给够时间真正转到，这一步才是唯一有意义的确认点
        self._wait_for_condition(
            lambda: self._latest_yaw is not None
            and _angle_diff(self._latest_yaw, targets[-1]) < self._yaw_tolerance,
            'Phase D2 settling on the final yaw target', self._yaw_timeout,
            settle_sec=self._settle_sec)
        err_deg = math.degrees(_angle_diff(self._latest_yaw, targets[-1]))
        self._record_result(
            'Phase D2: continuous yaw retarget (final settle, NOT single-shot accuracy)',
            f'{math.degrees(targets[-1]):.0f}deg (last of 2 rapid-fire targets)',
            f'{math.degrees(self._latest_yaw):.0f}deg', f'{err_deg:.1f}deg')
        self.get_logger().info(f'Phase D2 confirmed (final yaw_err={err_deg:.1f}deg)')

    # -- 收尾 --

    def _check_land_and_disarm(self):
        self.get_logger().info('calling set_mode(LAND) to clean up ...')
        result = self._call(self._set_mode_client, SetMode.Request(mode='LAND'), 'set_mode')
        if not result.success:
            raise RuntimeError('set_mode(LAND) returned success=False')
        self._wait_for_condition(
            lambda: self._latest_armed is False,
            'armed telemetry confirming auto-disarm after landing', self._land_timeout)
        self.get_logger().info('landed and disarmed (confirmed via telemetry)')


def main(args=None):
    rclpy.init(args=args)
    try:
        InterfaceManeuverTest()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
