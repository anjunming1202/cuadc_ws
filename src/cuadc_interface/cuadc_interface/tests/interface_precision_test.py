"""接口层精细机动测试：验证 Set Position Target（局部坐标精细调整）的
精度、连续追踪时的表现，并模拟"粗飞接近 + 精细对准 + 投放"这个真实任务
场景。跟 interface_maneuver_test.py 分开放——那边测的是 Go To
Location/Yaw 这类粗动作，这边测的是精细定位 + 投放机构，但用同一张
目标/实测/误差汇总表格式，方便互相对比精度。

`interface_node.py` 用的是 `MAV_FRAME_BODY_OFFSET_NED`——Set Position
Target 的 x/y/z 是**相对当前位置和当前朝向的偏移量**（前/右/下），飞控
收到指令那一刻用自己最新状态现算，不是这个测试脚本读一次遥测再加；这个
frame 下 yaw 也是相对量，固定发 0 就是"不转"，不会触发 ArduCopter 的
WP_YAW_BEHAVIOR。跟 Go To Location（绝对 GPS 坐标 + 默认自动朝向）刻意
区分开，见 docs/spec/04-interfaces.md"自动朝向行为"一节。

测试脚本里"前 N 米"这种量算的都是发给飞控的原始相对偏移量；用来验证
"有没有真的到位"的目标位置，是脚本自己另外用发送那一刻的当前位置+朝向
算出来的（`_body_offset_to_local`），只是核对用，不影响发给飞控的协议
本身。收尾回 home 用的是另一个接口 `~/cmd/position_target_local`
（`MAV_FRAME_LOCAL_NED`，绝对局部坐标，同样不自动转向）——home 在局部
坐标系里的位置是起飞后就记录好的，不需要临时算旋转。

  Setup 先粗飞 10m 建立一个任务点（Go To Location，绝对坐标，会自动
    转向飞行方向，这是它的预期行为，不是这个测试要验证的东西）。
  Phase A 单动作精度——Set Position Target 做一次小幅相对偏移调整，
    测误差。
  Phase B 累计误差——同一动作重复 4 次形成闭环，测首尾误差；顺带确认
    这几次纯 Set Position Target 指令期间朝向没被自动改动（呼应上面
    "自动朝向已关闭"）。
  Phase C 连续追踪——不等收敛就连续发新的相对偏移指令，确认新指令能
    正确顶替还没到位的旧指令（同 interface_maneuver_test.py 的压力
    测试套路，不代表精度；间隔刻意短于单次收敛时间，否则测不出"打断"
    这个效果）。
  Phase D 粗飞接近 + 精细对准 + 投放模拟——粗飞 10m 到虚拟投放区 ->
    Set Position Target 精细对准 -> Release Payload 触发投放（RC override 时序）
    -> 订阅 ~/status/drop 确认遥测真的反馈了投放执行（不只看 service success，
    同 interface_acceptance_test 里"goto_location 光看 success 不看
    真实位移"是同一类坑）。

所有精度测量都要求条件连续保持 settle_sec 才算数（同
interface_maneuver_test.py 的收敛判定），避免路过阈值的瞬时状态被
误判为收敛。

不在 launch 文件里，单独手动跑，同 interface_acceptance_test/
interface_maneuver_test。

用法：
  ros2 launch cuadc_interface interface_bringup.launch.py   # 另一个终端
  ros2 run cuadc_interface interface_precision_test
"""
import math
import time

import rclpy
from rclpy.node import Node

from cuadc_interface_msgs.msg import DropStatus, EkfStatus, PositionTarget
from cuadc_interface_msgs.srv import Arm, GotoLocation, ReleasePayload, SetMode, Takeoff
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Empty

_METERS_PER_DEGREE = 111320  # 粗略换算，纬度低误差小，够用（同 interface_maneuver_test）


def _yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a, b):
    d = (a - b + math.pi) % (2 * math.pi) - math.pi
    return abs(d)


# arm+takeoff 放行闸门话题（所有会飞的测试共用），同最终任务 task_start_trigger 套路。
START_TOPIC = '/cuadc_flight_test/start'

class InterfacePrecisionTest(Node):

    def __init__(self):
        super().__init__('interface_precision_test')
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('ekf_ready_timeout_sec', 60.0)
        self.declare_parameter('takeoff_altitude_m', 10.0)
        self.declare_parameter('altitude_tolerance_m', 3.0)
        self.declare_parameter('goto_tolerance_m', 2.0)
        self.declare_parameter('precision_tolerance_m', 0.5)
        self.declare_parameter('leg_timeout_sec', 30.0)
        self.declare_parameter('land_timeout_sec', 60.0)
        self.declare_parameter('settle_sec', 1.5)
        self.declare_parameter('yaw_stability_tolerance_deg', 5.0)
        self._service_timeout = self.get_parameter('service_timeout_sec').value
        self._ekf_ready_timeout = self.get_parameter('ekf_ready_timeout_sec').value
        self._takeoff_altitude = self.get_parameter('takeoff_altitude_m').value
        self._altitude_tolerance = self.get_parameter('altitude_tolerance_m').value
        self._goto_tolerance = self.get_parameter('goto_tolerance_m').value
        self._precision_tolerance = self.get_parameter('precision_tolerance_m').value
        self._leg_timeout = self.get_parameter('leg_timeout_sec').value
        self._land_timeout = self.get_parameter('land_timeout_sec').value
        self._settle_sec = self.get_parameter('settle_sec').value
        self._yaw_stability_tolerance_deg = self.get_parameter('yaw_stability_tolerance_deg').value

        self._arm_client = self.create_client(Arm, '/cuadc_interface/arm')
        self._set_mode_client = self.create_client(SetMode, '/cuadc_interface/set_mode')
        self._takeoff_client = self.create_client(Takeoff, '/cuadc_interface/takeoff')
        self._goto_client = self.create_client(GotoLocation, '/cuadc_interface/goto_location')
        self._drop_client = self.create_client(ReleasePayload, '/cuadc_interface/release_payload')
        self._position_target_pub = self.create_publisher(
            PositionTarget, '/cuadc_interface/cmd/position_target', 10)
        self._position_target_local_pub = self.create_publisher(
            PositionTarget, '/cuadc_interface/cmd/position_target_local', 10)

        self._latest_armed = None
        self._latest_ekf = None
        self._latest_position = None  # NavSatFix：goto 距离计算用
        self._latest_local_pose = None  # PoseStamped：x/y/z 是绝对局部 NED 坐标
        self._latest_yaw = None
        self._latest_drop_status = None
        self._results = []  # (name, target, achieved, error) 汇总表
        self.create_subscription(Bool, '/cuadc_interface/status/armed', self._on_armed, 10)
        self.create_subscription(
            EkfStatus, '/cuadc_interface/telemetry/ekf_status', self._on_ekf, 10)
        self.create_subscription(
            NavSatFix, '/cuadc_interface/telemetry/global_position', self._on_position, 10)
        self.create_subscription(
            PoseStamped, '/cuadc_interface/telemetry/local_position', self._on_local_pose, 10)
        self.create_subscription(Imu, '/cuadc_interface/telemetry/attitude', self._on_attitude, 10)
        self.create_subscription(
            DropStatus, '/cuadc_interface/status/drop', self._on_drop_status, 10)

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

        home_x, home_y, _ = self._local_xyz()

        # -- Setup：粗飞 10m 建立任务点（Go To Location，会自动转向） --
        self._goto_forward(10.0, 'Setup: Go To Location (forward 10m, establish task point)')
        heading_before_precision = self._latest_yaw

        # -- Phase A：Set Position Target 单动作精度 --
        self._position_target_full(
            3.0, 2.0, 0.0, 'Phase A: Set Position Target (3m forward, 2m right)')

        # -- Phase B：累计误差（闭环）+ 确认自动朝向已关闭 --
        self._check_position_target_cumulative()
        self._check_heading_held(heading_before_precision)

        # -- Phase C：连续追踪，不等收敛 --
        self._check_continuous_position_target()

        # -- Phase D：粗飞接近 + 精细对准 + 投放模拟 --
        self._check_align_and_drop()

        # -- 收尾：闭环回归 home，走局部绝对坐标（不经过 Go To Location，
        # 全程不自动转向）——home 在局部坐标系里位置是已知的，不用像机身
        # 相对偏移那样临时算旋转 --
        self._position_target_local_full(
            home_x, home_y, -self._takeoff_altitude,
            'Return home (position, local-absolute, no auto-yaw)')
        self._check_land_and_disarm()

        self._print_summary()
        self.get_logger().info(
            'interface precision test PASSED: fine-adjustment accuracy, cumulative drift, '
            'auto-yaw lock, continuous retargeting, and align-then-drop all confirmed')
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

    def _on_drop_status(self, msg):
        self._latest_drop_status = msg

    # -- 辅助方法 --

    def _relative_altitude(self):
        return -self._latest_local_pose.pose.position.z

    def _local_xyz(self):
        p = self._latest_local_pose.pose.position
        return p.x, p.y, p.z

    def _distance_xy(self, x, y):
        cx, cy, _ = self._local_xyz()
        return math.hypot(cx - x, cy - y)

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
        # 状态，不是真正收敛——条件要连续保持 settle_sec 才算数，中途掉
        # 出阈值就重新计时。
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
            f'    ros2 topic pub -t 5 -r 2 {START_TOPIC} std_msgs/msg/Empty "{{}}"')
        self._wait_for_condition(
            lambda: self._start_triggered, 'flight start trigger', self._start_timeout)
        self.get_logger().info('start trigger received -> arming + taking off')

    def _wait_for_services(self):
        for name, client in (
            ('arm', self._arm_client),
            ('set_mode', self._set_mode_client),
            ('takeoff', self._takeoff_client),
            ('goto_location', self._goto_client),
            ('release_payload', self._drop_client),
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

    # -- Go To Location：粗飞到位，等到 100% 收敛 --

    def _goto_forward(self, distance_m, label):
        # Go To Location 是绝对 GPS 坐标，"前"不能直接当成地理正北——按
        # 当前朝向投影出一个目标经纬度（一次性算，不是链式累加，不会有
        # 局部坐标那种累计漂移问题）。
        yaw = self._latest_yaw
        lat_rad = math.radians(self._latest_position.latitude)
        north_m = distance_m * math.cos(yaw)
        east_m = distance_m * math.sin(yaw)
        lat = self._latest_position.latitude + north_m / _METERS_PER_DEGREE
        lon = self._latest_position.longitude + east_m / (_METERS_PER_DEGREE * math.cos(lat_rad))
        self._goto_full(lat, lon, self._takeoff_altitude, label)

    def _goto_full(self, lat, lon, alt, label):
        start_dist = self._distance_to(lat, lon)
        start_alt = self._relative_altitude()
        self.get_logger().info(
            f'{label}: goto lat={lat:.7f} lon={lon:.7f} alt={alt:.1f} (dist={start_dist:.1f}m) ...')
        result = self._call(
            self._goto_client, GotoLocation.Request(lat=lat, lon=lon, alt=alt), label)
        if not result.success:
            raise RuntimeError(f'{label}: goto_location returned success=False')

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
            if self._distance_to(lat, lon) <= self._goto_tolerance:
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

    # -- Set Position Target：body-relative 偏移（前/右/下），等到 100% 收敛 --

    def _body_offset_to_local(self, cur_x, cur_y, cur_z, yaw, dx, dy, dz):
        # 跟 interface_node.py 里 ArduCopter 对 MAV_FRAME_BODY_OFFSET_NED
        # 的处理同一套旋转公式，只是这边算出来仅用于核对遥测，不发给飞控。
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        north = dx * cos_yaw - dy * sin_yaw
        east = dx * sin_yaw + dy * cos_yaw
        return cur_x + north, cur_y + east, cur_z + dz

    def _position_target_full(self, dx, dy, dz, label):
        cur_x, cur_y, cur_z = self._local_xyz()
        target_x, target_y, target_z = self._body_offset_to_local(
            cur_x, cur_y, cur_z, self._latest_yaw, dx, dy, dz)
        self.get_logger().info(
            f'{label}: body offset forward={dx:.1f}m right={dy:.1f}m down={dz:.1f}m '
            f'(expect local x={target_x:.2f} y={target_y:.2f} z={target_z:.2f}) ...')
        self._position_target_pub.publish(PositionTarget(x=dx, y=dy, z=dz))
        self._wait_for_condition(
            lambda: self._distance_xy(target_x, target_y) <= self._precision_tolerance,
            f'{label} reaching target', self._leg_timeout, settle_sec=self._settle_sec)
        pos_err = self._distance_xy(target_x, target_y)
        alt_err = -(self._latest_local_pose.pose.position.z - target_z)
        self._record_result(
            label, f'forward={dx:.1f}m right={dy:.1f}m',
            f'pos_err={pos_err:.2f}m', f'{pos_err:.2f}m (alt_err={alt_err:+.2f}m)')
        self.get_logger().info(f'{label}: confirmed (pos_err={pos_err:.2f}m, alt_err={alt_err:+.2f}m)')

    # -- Set Position Target (Local)：绝对局部坐标，等到 100% 收敛 --

    def _position_target_local_full(self, target_x, target_y, target_z, label):
        self.get_logger().info(
            f'{label}: local absolute target x={target_x:.2f} y={target_y:.2f} '
            f'z={target_z:.2f} ...')
        self._position_target_local_pub.publish(
            PositionTarget(x=target_x, y=target_y, z=target_z))
        self._wait_for_condition(
            lambda: self._distance_xy(target_x, target_y) <= self._precision_tolerance,
            f'{label} reaching target', self._leg_timeout, settle_sec=self._settle_sec)
        pos_err = self._distance_xy(target_x, target_y)
        alt_err = -(self._latest_local_pose.pose.position.z - target_z)
        self._record_result(
            label, f'x={target_x:.2f}m y={target_y:.2f}m',
            f'pos_err={pos_err:.2f}m', f'{pos_err:.2f}m (alt_err={alt_err:+.2f}m)')
        self.get_logger().info(f'{label}: confirmed (pos_err={pos_err:.2f}m, alt_err={alt_err:+.2f}m)')

    # -- Phase B：累计误差 --

    def _check_position_target_cumulative(self):
        d = 2.0  # 每条腿 2m，精细调整幅度
        start_x, start_y, _ = self._local_xyz()
        legs = [
            (d, 0.0, 'Phase B leg1 (forward)'),
            (0.0, d, 'Phase B leg2 (right)'),
            (-d, 0.0, 'Phase B leg3 (backward)'),
            (0.0, -d, 'Phase B leg4 (left, back to loop start)'),
        ]
        for dx, dy, label in legs:
            self._position_target_full(dx, dy, 0.0, label)
        final_err = self._distance_xy(start_x, start_y)
        self._record_result(
            'Phase B: Set Position Target cumulative (4-leg loop)',
            'back to loop start', f'pos_err={final_err:.2f}m', f'{final_err:.2f}m')
        self.get_logger().info(f'Phase B cumulative position error: {final_err:.2f}m')

    def _check_heading_held(self, expected_yaw):
        # Set Position Target 已经在 interface_node.py 里锁定了 yaw（见
        # 04-interfaces.md"自动朝向行为"一节），Phase A/B 全程只发这个
        # 指令，朝向应该跟锁定前几乎一样——这里把"应该不变"变成实测断言。
        err_deg = math.degrees(_angle_diff(self._latest_yaw, expected_yaw))
        self._record_result(
            'Set Position Target auto-yaw disabled (heading held through Phase A/B)',
            f'{math.degrees(expected_yaw):.0f}deg', f'{math.degrees(self._latest_yaw):.0f}deg',
            f'{err_deg:.1f}deg')
        if err_deg > self._yaw_stability_tolerance_deg:
            raise RuntimeError(
                f'heading drifted {err_deg:.1f}deg during Set Position Target-only phases '
                f'(tolerance {self._yaw_stability_tolerance_deg:.0f}deg) -- yaw lock not working')
        self.get_logger().info(f'heading held through Phase A/B (drift={err_deg:.1f}deg)')

    # -- Phase C：连续追踪，不等收敛，确认新指令能顶替还没到位的旧指令 --

    def _check_continuous_position_target(self):
        # 每个目标都是相对偏移，没法预先在 Python 这边算出一串绝对路径点
        # 来验证；"打断"是不是真的发生了不能只靠猜时间窗口够不够短——直接
        # 在发下一条指令前量一下离上一条目标还差多少，把"确实还没到位就被
        # 打断"变成日志里能看到的实测数据，不是靠假设。
        self.get_logger().info(
            'Phase C (continuous Set Position Target retargeting, no settle wait between calls) ...')
        prev_target = None
        for dx, dy in ((1.0, 0.0), (0.0, 1.0), (1.0, -1.0)):
            if prev_target is not None:
                gap = self._distance_xy(*prev_target[:2])
                self.get_logger().info(
                    f'Phase C: {gap:.2f}m from previous target when sending the next one '
                    '(>0 confirms it was interrupted mid-transit)')
            cur_x, cur_y, cur_z = self._local_xyz()
            prev_target = self._body_offset_to_local(
                cur_x, cur_y, cur_z, self._latest_yaw, dx, dy, 0.0)
            self._position_target_pub.publish(PositionTarget(x=dx, y=dy, z=0.0))
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.3)
        gap = self._distance_xy(*prev_target[:2])
        self.get_logger().info(
            f'Phase C: {gap:.2f}m from previous target when sending the final one')
        self._position_target_full(
            1.0, 0.0, 0.0,
            'Phase C: continuous retarget (final settle, NOT single-shot accuracy)')

    # -- Phase D：粗飞到投放区 + 精细对准 + 投放，确认遥测真的反馈了状态 --

    def _check_align_and_drop(self):
        self._goto_forward(
            10.0, 'Phase D: Go To Location (forward 10m, coarse approach to drop zone)')

        self._position_target_full(
            1.0, 0.5, 0.0, 'Phase D: Set Position Target (align, forward-right)')
        self._position_target_full(
            1.0, 0.0, 1.5, 'Phase D: Set Position Target (descend, forward-down)')

        self._release_payload_and_confirm('Phase D: Release Payload (drop)')

    def _release_payload_and_confirm(self, label):
        # release_payload 会跑完整段投放 RC override 时序才返回（阻塞约 drop_run_s），
        # 完成后接口层发 ~/status/drop（actuated=True）。见 docs/spec/adr/0007。
        self._latest_drop_status = None
        self.get_logger().info(f'{label}: calling release_payload() ...')
        result = self._call(self._drop_client, ReleasePayload.Request(), label)
        if not result.success:
            raise RuntimeError(f'{label}: release_payload returned success=False: {result.message}')
        self._wait_for_condition(
            lambda: self._latest_drop_status is not None and self._latest_drop_status.actuated,
            f'{label} drop status telemetry confirming actuated', self._service_timeout)
        self._record_result(label, 'actuated=True',
                            f'actuated={self._latest_drop_status.actuated}', 'confirmed via telemetry')
        self.get_logger().info(f'{label}: confirmed via telemetry')

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
        InterfacePrecisionTest()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
