"""接口层速度测试：验证 Set Velocity（`~/cmd/velocity`）最基本的两件
事——发速度真的会让飞机动，停止发送后会安全悬停、不会沿着最后的速度
继续漂。比另外三个测试脚本简单很多，是一次性快速验证，不是常驻精度
测试（Set Velocity 是纯速度 reference，没有位置目标，谈不上"精度"）。

流程：起飞 -> 持续发一段时间的水平速度，确认遥测真的测到了位移（不是
只看话题发出去了）-> 停止发送，观察一段时间确认飞机悬停住 -> 降落。

Set Velocity 用的是 `MAV_FRAME_BODY_OFFSET_NED`（机身系），这里给的
速度方向是机头前方，不是地理正北。

不在 launch 文件里，单独手动跑，同其它三个测试脚本。

用法：
  ros2 launch cuadc_interface interface_bringup.launch.py   # 另一个终端
  ros2 run cuadc_interface interface_velocity_test
"""
import math
import time

import rclpy
from rclpy.node import Node

from cuadc_interface_msgs.msg import EkfStatus
from cuadc_interface_msgs.srv import Arm, SetMode, Takeoff
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool, Empty


# arm+takeoff 放行闸门话题（所有会飞的测试共用），同最终任务 task_start_trigger 套路。
START_TOPIC = '/cuadc_flight_test/start'

class InterfaceVelocityTest(Node):

    def __init__(self):
        super().__init__('interface_velocity_test')
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('ekf_ready_timeout_sec', 60.0)
        self.declare_parameter('takeoff_altitude_m', 10.0)
        self.declare_parameter('velocity_forward_ms', 2.0)
        self.declare_parameter('velocity_duration_sec', 4.0)
        self.declare_parameter('min_displacement_m', 3.0)
        self.declare_parameter('speed_zero_threshold_ms', 0.3)
        self.declare_parameter('speed_settle_sec', 1.5)
        self.declare_parameter('speed_zero_timeout_sec', 8.0)
        self.declare_parameter('post_stop_drift_window_sec', 2.0)
        self.declare_parameter('drift_tolerance_m', 1.0)
        self.declare_parameter('land_timeout_sec', 60.0)
        self._service_timeout = self.get_parameter('service_timeout_sec').value
        self._ekf_ready_timeout = self.get_parameter('ekf_ready_timeout_sec').value
        self._takeoff_altitude = self.get_parameter('takeoff_altitude_m').value
        self._velocity_forward = self.get_parameter('velocity_forward_ms').value
        self._velocity_duration = self.get_parameter('velocity_duration_sec').value
        self._min_displacement = self.get_parameter('min_displacement_m').value
        self._speed_zero_threshold = self.get_parameter('speed_zero_threshold_ms').value
        self._speed_settle_sec = self.get_parameter('speed_settle_sec').value
        self._speed_zero_timeout = self.get_parameter('speed_zero_timeout_sec').value
        self._post_stop_drift_window = self.get_parameter('post_stop_drift_window_sec').value
        self._drift_tolerance = self.get_parameter('drift_tolerance_m').value
        self._land_timeout = self.get_parameter('land_timeout_sec').value

        self._arm_client = self.create_client(Arm, '/cuadc_interface/arm')
        self._set_mode_client = self.create_client(SetMode, '/cuadc_interface/set_mode')
        self._takeoff_client = self.create_client(Takeoff, '/cuadc_interface/takeoff')
        self._velocity_pub = self.create_publisher(
            TwistStamped, '/cuadc_interface/cmd/velocity', 10)

        self._latest_armed = None
        self._latest_ekf = None
        self._latest_local_pose = None  # PoseStamped：x/y/z 是绝对局部 NED 坐标
        self._latest_velocity = None  # TwistStamped：判断"真的停了"要看这个，不是位置
        self.create_subscription(Bool, '/cuadc_interface/status/armed', self._on_armed, 10)
        self.create_subscription(
            EkfStatus, '/cuadc_interface/telemetry/ekf_status', self._on_ekf, 10)
        self.create_subscription(
            PoseStamped, '/cuadc_interface/telemetry/local_position', self._on_local_pose, 10)
        self.create_subscription(
            TwistStamped, '/cuadc_interface/telemetry/velocity', self._on_velocity_telemetry, 10)

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

        self._check_velocity_moves()
        self._check_velocity_stops_cleanly()
        self._check_land_and_disarm()

        self.get_logger().info('interface velocity test PASSED: moves under velocity command, '
                                'holds position cleanly once stopped')
        rclpy.shutdown()

    # -- 遥测回调 --

    def _on_armed(self, msg):
        self._latest_armed = msg.data

    def _on_ekf(self, msg):
        self._latest_ekf = msg

    def _on_local_pose(self, msg):
        self._latest_local_pose = msg

    def _on_velocity_telemetry(self, msg):
        self._latest_velocity = msg

    # -- 辅助方法 --

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

    def _wait_for_condition(self, predicate, description, timeout, settle_sec=0.0):
        # 单次轮询命中阈值就返回可能逮到的是瞬时状态，不是真正稳住——
        # 条件要连续保持 settle_sec 才算数，中途掉出阈值就重新计时。
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
            lambda: self._latest_local_pose is not None, 'local_position samples', 10.0)
        self.get_logger().info(f'calling takeoff(alt={self._takeoff_altitude}) ...')
        result = self._call(
            self._takeoff_client, Takeoff.Request(alt=self._takeoff_altitude), 'takeoff')
        if not result.success:
            raise RuntimeError('takeoff returned success=False')
        self._wait_for_condition(
            lambda: self._relative_altitude() >= self._takeoff_altitude - 1.0,
            'altitude telemetry confirming climb', 30.0)
        self.get_logger().info(f'takeoff confirmed (alt={self._relative_altitude():.2f}m)')

    # -- Set Velocity：发速度真的会动，停止发送真的会停 --

    def _check_velocity_moves(self):
        start_x, start_y = self._local_xy()
        self.get_logger().info(
            f'sending {self._velocity_forward:.1f}m/s forward for {self._velocity_duration:.1f}s ...')
        twist = TwistStamped()
        twist.twist.linear.x = self._velocity_forward
        deadline = time.monotonic() + self._velocity_duration
        while time.monotonic() < deadline:
            self._velocity_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.2)
        moved = self._distance_xy(start_x, start_y)
        if moved < self._min_displacement:
            raise RuntimeError(
                f'only moved {moved:.2f}m after {self._velocity_duration:.1f}s of velocity '
                f'command -- expected at least {self._min_displacement:.1f}m')
        self.get_logger().info(f'velocity confirmed moving the vehicle ({moved:.2f}m displacement)')

    def _check_velocity_stops_cleanly(self):
        # 停止发送后飞机还在按 2m/s 飞——ArduCopter GUID_TIMEOUT（默认
        # 3s）到期前会继续按惯性走完，之后还要花时间刹车减速，这段时间
        # 内位置本来就会继续变化，不是没停下来。真正该测的是"速度有没有
        # 真的降到接近 0 并稳住"，稳住之后才去看位置是否还在漂——不能拿
        # "停止发送指令那一刻起位置有没有变化"当标准（那一刻飞机本来就
        # 还在用 2m/s 飞）。
        self.get_logger().info(
            f'stopped sending velocity, waiting for speed to settle below '
            f'{self._speed_zero_threshold:.1f}m/s ...')
        self._wait_for_condition(
            lambda: self._latest_velocity is not None
            and self._horizontal_speed() < self._speed_zero_threshold,
            'speed dropping to near zero and holding', self._speed_zero_timeout,
            settle_sec=self._speed_settle_sec)

        settle_x, settle_y = self._local_xy()
        self.get_logger().info(
            f'speed settled, watching {self._post_stop_drift_window:.1f}s more to confirm '
            'position actually holds ...')
        deadline = time.monotonic() + self._post_stop_drift_window
        max_drift = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            max_drift = max(max_drift, self._distance_xy(settle_x, settle_y))
        if max_drift > self._drift_tolerance:
            raise RuntimeError(
                f'drifted {max_drift:.2f}m after speed settled (tolerance '
                f'{self._drift_tolerance:.1f}m) -- expected safe hover, not continued drift')
        self.get_logger().info(f'confirmed safe hover after stopping (max drift={max_drift:.2f}m)')

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
        InterfaceVelocityTest()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
