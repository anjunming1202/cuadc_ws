"""接口层验收测试：驱动 cuadc_interface 自己对外暴露的 ROS2 契约（服务/
话题），不直接说 MAVLink。跟 cuadc_sim 的 sitl_smoke_test/
flight_acceptance_test 测的是不同层——那两个验证的是 MAVLink 协议本身，
这个验证的是接口层这一层"翻译"对不对（发的服务请求有没有真的让飞控
状态按预期变化，遥测有没有如实报回来）。

假设 cuadc_interface 节点已经跑起来并连上了 SITL/真飞控——不负责起
SITL、不负责起接口层节点本身，同 cuadc_sim 两个 bringup launch 文件
"只管搭世界，不管在世界里做什么"的原则一致（见
src/cuadc_sim/README.md）。

用法：
  ros2 launch cuadc_interface interface_bringup.launch.py   # 另一个终端
  ros2 run cuadc_interface interface_acceptance_test
"""
import math
import time

import rclpy
from rclpy.node import Node

from cuadc_interface_msgs.msg import EkfStatus
from cuadc_interface_msgs.srv import Arm, GotoLocation, ReleasePayload, SetMode, Takeoff
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String, Empty


# arm+takeoff 放行闸门话题（所有会飞的测试共用），同最终任务 task_start_trigger 套路。
START_TOPIC = '/cuadc_flight_test/start'

class InterfaceAcceptanceTest(Node):

    def __init__(self):
        super().__init__('interface_acceptance_test')
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('ekf_ready_timeout_sec', 60.0)
        self.declare_parameter('telemetry_timeout_sec', 10.0)
        self.declare_parameter('takeoff_altitude_m', 10.0)
        self.declare_parameter('takeoff_timeout_sec', 30.0)
        self.declare_parameter('land_timeout_sec', 60.0)
        self._service_timeout = self.get_parameter('service_timeout_sec').value
        self._ekf_ready_timeout = self.get_parameter('ekf_ready_timeout_sec').value
        self._telemetry_timeout = self.get_parameter('telemetry_timeout_sec').value
        self._takeoff_altitude = self.get_parameter('takeoff_altitude_m').value
        self._takeoff_timeout = self.get_parameter('takeoff_timeout_sec').value
        self._land_timeout = self.get_parameter('land_timeout_sec').value

        self._arm_client = self.create_client(Arm, '/cuadc_interface/arm')
        self._set_mode_client = self.create_client(SetMode, '/cuadc_interface/set_mode')
        self._takeoff_client = self.create_client(Takeoff, '/cuadc_interface/takeoff')
        self._goto_client = self.create_client(GotoLocation, '/cuadc_interface/goto_location')
        self._drop_client = self.create_client(ReleasePayload, '/cuadc_interface/release_payload')

        self._latest_mode = None
        self._latest_armed = None
        self._latest_ekf = None
        self._latest_position = None
        self.create_subscription(String, '/cuadc_interface/status/mode', self._on_mode, 10)
        self.create_subscription(Bool, '/cuadc_interface/status/armed', self._on_armed, 10)
        self.create_subscription(
            EkfStatus, '/cuadc_interface/telemetry/ekf_status', self._on_ekf, 10)
        self.create_subscription(
            NavSatFix, '/cuadc_interface/telemetry/global_position', self._on_position, 10)

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
        self._check_goto_location()
        self._check_release_payload()
        self._check_land_and_disarm()

        self.get_logger().info(
            'interface acceptance test PASSED: set_mode/arm/takeoff/goto_location/'
            'release_payload/land all confirmed against a live vehicle via the ROS2 contract')
        rclpy.shutdown()

    def _on_mode(self, msg):
        self._latest_mode = msg.data

    def _on_armed(self, msg):
        self._latest_armed = msg.data

    def _on_ekf(self, msg):
        self._latest_ekf = msg

    def _on_position(self, msg):
        self._latest_position = msg

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
            self.get_logger().info(f'waiting for {name} service ...')
            if not client.wait_for_service(timeout_sec=self._service_timeout):
                raise RuntimeError(
                    f'{name} service not available within '
                    f'{self._service_timeout:.0f}s -- is cuadc_interface running '
                    'and connected?')

    def _wait_for_condition(self, predicate, description, timeout=None):
        deadline = time.monotonic() + (timeout or self._telemetry_timeout)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            if predicate():
                return
        raise RuntimeError(f'{description} did not happen within timeout')

    def _wait_for_ekf_ready(self):
        self.get_logger().info(
            f'waiting for EKF/GPS ready (timeout {self._ekf_ready_timeout:.0f}s) ...')
        self._wait_for_condition(
            lambda: self._latest_ekf is not None and self._latest_ekf.healthy,
            'EKF/GPS becoming healthy', timeout=self._ekf_ready_timeout)
        self.get_logger().info('EKF/GPS ready')

    def _call(self, client, request, name):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._service_timeout)
        if not future.done():
            raise RuntimeError(f'{name} call timed out')
        return future.result()

    def _check_set_mode(self):
        self.get_logger().info('calling set_mode(GUIDED) ...')
        result = self._call(self._set_mode_client, SetMode.Request(mode='GUIDED'), 'set_mode')
        if not result.success:
            raise RuntimeError('set_mode(GUIDED) returned success=False')
        self._wait_for_condition(
            lambda: self._latest_mode == 'GUIDED', 'mode telemetry confirming GUIDED')
        self.get_logger().info('set_mode(GUIDED) confirmed via telemetry')

    def _check_arm(self):
        self.get_logger().info('calling arm(True) ...')
        result = self._call(self._arm_client, Arm.Request(arm=True), 'arm')
        if not result.success:
            raise RuntimeError(f'arm(True) returned success=False: {result.message}')
        self._wait_for_condition(
            lambda: self._latest_armed is True, 'armed telemetry confirming True')
        self.get_logger().info('arm(True) confirmed via telemetry')

    def _check_takeoff(self):
        # 2026-07-15 SITL 实测确认：光发 Go To Location/Set Velocity 不会
        # 让停在地面的飞机爬升，必须先调这个——见 04-interfaces.md。
        self._wait_for_condition(
            lambda: self._latest_position is not None, 'a global_position sample')
        base_alt = self._latest_position.altitude
        self.get_logger().info(
            f'calling takeoff(alt={self._takeoff_altitude}) from base_alt={base_alt:.2f} ...')
        result = self._call(self._takeoff_client, Takeoff.Request(alt=self._takeoff_altitude), 'takeoff')
        if not result.success:
            raise RuntimeError('takeoff returned success=False')
        self._wait_for_condition(
            lambda: (self._latest_position.altitude - base_alt) >=
            self._takeoff_altitude - 1.0,
            'altitude telemetry confirming climb', timeout=self._takeoff_timeout)
        self.get_logger().info('takeoff confirmed via telemetry (climbed to target altitude)')

    def _check_goto_location(self):
        # SET_POSITION_TARGET_GLOBAL_INT 没有协议层 ack，service 返回
        # success=True 只代表"发出去了"，不代表飞控真的执行了，所以要靠
        # 遥测算实际移动距离确认，不能只看服务返回值就往下走——距离本身
        # 选多大不是关键，等不等飞机真的飞到才是关键。目标点相对当前
        # 位置算偏移（不写死绝对坐标，换起飞点/场地世界文件不用改这个
        # 脚本），高度维持跟起飞一致，不同时变高度，避免爬升掩盖水平
        # 位移的判断。假设已经在空中（先 _check_takeoff）——地面上发这个
        # 不生效。
        self._wait_for_condition(
            lambda: self._latest_position is not None, 'a global_position sample')
        pos = self._latest_position
        base_lat, base_lon = pos.latitude, pos.longitude
        target_lat = base_lat + 0.000045  # ~5m north
        target_lon = base_lon
        self.get_logger().info(
            f'calling goto_location(lat={target_lat:.7f}, lon={target_lon:.7f}, '
            f'alt={self._takeoff_altitude}) ...')
        request = GotoLocation.Request(
            lat=target_lat, lon=target_lon, alt=self._takeoff_altitude)
        result = self._call(self._goto_client, request, 'goto_location')
        if not result.success:
            raise RuntimeError('goto_location returned success=False')

        def distance_from_base():
            dlat = self._latest_position.latitude - base_lat
            dlon = self._latest_position.longitude - base_lon
            return math.hypot(dlat, dlon) * 111320  # 粗略换算成米

        self._wait_for_condition(
            lambda: distance_from_base() >= 4.0,
            'position telemetry confirming the vehicle actually moved ~5m',
            timeout=15.0)
        self.get_logger().info(
            f'goto_location confirmed via telemetry (moved '
            f'{distance_from_base():.1f}m from base)')

    def _check_release_payload(self):
        # 在空中、已解锁状态调用（release_payload 有 armed 守卫，见 ADR-0007）；服务会跑完
        # 整段投放 RC override 时序才返回，这里会阻塞约 drop_run_s。
        self.get_logger().info('calling release_payload() ...')
        result = self._call(self._drop_client, ReleasePayload.Request(), 'release_payload')
        if not result.success:
            raise RuntimeError(f'release_payload returned success=False: {result.message}')
        self.get_logger().info('release_payload accepted')

    def _check_land_and_disarm(self):
        # 空中直接 arm(False) 等于断电摔机——flight_acceptance_test.py 把
        # 这种情况当失败条件，不是合法收尾方式。必须先 LAND，落地后
        # ArduCopter 会自己 disarm，靠遥测确认，不主动发 disarm 指令。
        self.get_logger().info('calling set_mode(LAND) to clean up ...')
        result = self._call(self._set_mode_client, SetMode.Request(mode='LAND'), 'set_mode')
        if not result.success:
            raise RuntimeError('set_mode(LAND) returned success=False')
        self._wait_for_condition(
            lambda: self._latest_armed is False,
            'armed telemetry confirming auto-disarm after landing',
            timeout=self._land_timeout)
        self.get_logger().info('landed and disarmed (confirmed via telemetry)')


def main(args=None):
    rclpy.init(args=args)
    try:
        InterfaceAcceptanceTest()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
