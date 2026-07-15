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
import time

import rclpy
from rclpy.node import Node

from cuadc_interface_msgs.msg import EkfStatus
from cuadc_interface_msgs.srv import Arm, GotoLocation, SetMode, SetRelay
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String


class InterfaceAcceptanceTest(Node):

    def __init__(self):
        super().__init__('interface_acceptance_test')
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('ekf_ready_timeout_sec', 60.0)
        self.declare_parameter('telemetry_timeout_sec', 10.0)
        self._service_timeout = self.get_parameter('service_timeout_sec').value
        self._ekf_ready_timeout = self.get_parameter('ekf_ready_timeout_sec').value
        self._telemetry_timeout = self.get_parameter('telemetry_timeout_sec').value

        self._arm_client = self.create_client(Arm, '/cuadc_interface/arm')
        self._set_mode_client = self.create_client(SetMode, '/cuadc_interface/set_mode')
        self._goto_client = self.create_client(GotoLocation, '/cuadc_interface/goto_location')
        self._relay_client = self.create_client(SetRelay, '/cuadc_interface/set_relay')

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

        self._wait_for_services()
        self._wait_for_ekf_ready()
        self._check_set_mode()
        self._check_arm()
        self._check_goto_location()
        self._check_set_relay()
        self._check_disarm()

        self.get_logger().info(
            'interface acceptance test PASSED: set_mode/arm/goto_location/'
            'set_relay all confirmed against a live vehicle via the ROS2 contract')
        rclpy.shutdown()

    def _on_mode(self, msg):
        self._latest_mode = msg.data

    def _on_armed(self, msg):
        self._latest_armed = msg.data

    def _on_ekf(self, msg):
        self._latest_ekf = msg

    def _on_position(self, msg):
        self._latest_position = msg

    def _wait_for_services(self):
        for name, client in (
            ('arm', self._arm_client),
            ('set_mode', self._set_mode_client),
            ('goto_location', self._goto_client),
            ('set_relay', self._relay_client),
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

    def _check_goto_location(self):
        # SET_POSITION_TARGET_GLOBAL_INT 没有协议层 ack（04-interfaces.md
        # 已经标了这一点），这里只能确认"服务返回成功"，不能像
        # set_mode/arm 那样靠遥测反向确认真的到没到。目标点相对当前位置
        # 算一个小偏移（同 cuadc_sim/sitl_smoke_test.py 的做法），不写死
        # 绝对坐标——这样换一个 SITL 起飞点/换场地世界文件都不用改这个脚本。
        self._wait_for_condition(
            lambda: self._latest_position is not None, 'a global_position sample')
        pos = self._latest_position
        target_lat = pos.latitude + 0.00003  # ~3m north
        target_lon = pos.longitude
        self.get_logger().info(
            f'calling goto_location(lat={target_lat:.7f}, lon={target_lon:.7f}, alt=20) ...')
        request = GotoLocation.Request(lat=target_lat, lon=target_lon, alt=20.0)
        result = self._call(self._goto_client, request, 'goto_location')
        if not result.success:
            raise RuntimeError('goto_location returned success=False')
        self.get_logger().info(
            'goto_location accepted (no protocol-level ack to confirm further)')

    def _check_set_relay(self):
        self.get_logger().info('calling set_relay(channel=0, state=True) ...')
        request = SetRelay.Request(channel=0, state=True)
        result = self._call(self._relay_client, request, 'set_relay')
        if not result.success:
            raise RuntimeError('set_relay returned success=False')
        self.get_logger().info('set_relay accepted')

    def _check_disarm(self):
        self.get_logger().info('calling arm(False) to clean up ...')
        result = self._call(self._arm_client, Arm.Request(arm=False), 'arm')
        if not result.success:
            raise RuntimeError('disarm returned success=False')
        self._wait_for_condition(
            lambda: self._latest_armed is False, 'armed telemetry confirming False')
        self.get_logger().info('disarm confirmed via telemetry')


def main(args=None):
    rclpy.init(args=args)
    try:
        InterfaceAcceptanceTest()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
