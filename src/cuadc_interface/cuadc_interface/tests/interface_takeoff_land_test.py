"""接口层起降测试（最小 smoke）：树莓派经接口层驱动飞控完成
「解锁 → 起飞到 H → 悬停观察 → 降落 → 自动上锁」这一条最短链路。

现场飞行的第一步就跑这个：先确认 Pi 真能指挥飞控起降、遥测能如实报回，
再上 interface_flight_suite_test 那套全向/速度/高度的完整合集。不做任何
水平移动，飞机只在起飞点正上方上下——风险最低。飞手 RC 全程待命。

高度用相对起飞点的高度（`local_position` 的 `-z`），不用海拔，见
docs/spec/04-interfaces.md「高度基准」。sim/real 同一节点，连接目标由
各自的启动脚本写死（run_flight_check_sim/real.sh）。

放行闸门：起来后停在 arm **之前**（桨不转）、等放行，另一 SSH 发
`ros2 topic pub -1 /cuadc_flight_test/start std_msgs/msg/Empty "{}"` 才 arm+起飞。
sim 快速迭代可 `-p wait_for_start:=false` 跳过。

不在 launch 里，单独手动跑（同其它接口测试节点）：
  ros2 launch cuadc_interface interface_bringup.launch.py   # 另一个终端
  ros2 run cuadc_interface interface_takeoff_land_test
"""
import sys
import time

import rclpy
from rclpy.node import Node

from cuadc_interface_msgs.msg import EkfStatus
from cuadc_interface_msgs.srv import Arm, SetMode, Takeoff
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty

# arm+takeoff 前的显式放行闸门话题（两个飞行测试节点共用），同最终任务的
# task_start_trigger 套路：节点起来先停这里、桨不转，人在另一 SSH 发信号才 arm+起飞。
START_TOPIC = '/cuadc_flight_test/start'


class InterfaceTakeoffLandTest(Node):

    def __init__(self):
        super().__init__('interface_takeoff_land_test')
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('ekf_ready_timeout_sec', 60.0)
        # arm+takeoff 放行闸门：默认开(现场安全)。sim 快速迭代可 -p wait_for_start:=false 直接飞。
        self.declare_parameter('wait_for_start', True)
        self.declare_parameter('start_timeout_sec', 600.0)
        self.declare_parameter('takeoff_altitude_m', 5.0)
        self.declare_parameter('takeoff_timeout_sec', 30.0)
        self.declare_parameter('altitude_tolerance_m', 0.5)
        self.declare_parameter('hover_hold_sec', 8.0)
        self.declare_parameter('land_timeout_sec', 60.0)
        self._service_timeout = self.get_parameter('service_timeout_sec').value
        self._ekf_ready_timeout = self.get_parameter('ekf_ready_timeout_sec').value
        self._wait_for_start_enabled = self.get_parameter('wait_for_start').value
        self._start_timeout = self.get_parameter('start_timeout_sec').value
        self._takeoff_altitude = self.get_parameter('takeoff_altitude_m').value
        self._takeoff_timeout = self.get_parameter('takeoff_timeout_sec').value
        self._altitude_tolerance = self.get_parameter('altitude_tolerance_m').value
        self._hover_hold = self.get_parameter('hover_hold_sec').value
        self._land_timeout = self.get_parameter('land_timeout_sec').value

        self._arm_client = self.create_client(Arm, '/cuadc_interface/arm')
        self._set_mode_client = self.create_client(SetMode, '/cuadc_interface/set_mode')
        self._takeoff_client = self.create_client(Takeoff, '/cuadc_interface/takeoff')

        self._latest_armed = None
        self._latest_ekf = None
        self._latest_local_pose = None
        self._start_triggered = False
        self.create_subscription(Bool, '/cuadc_interface/status/armed', self._on_armed, 10)
        self.create_subscription(
            EkfStatus, '/cuadc_interface/telemetry/ekf_status', self._on_ekf, 10)
        self.create_subscription(
            PoseStamped, '/cuadc_interface/telemetry/local_position', self._on_local_pose, 10)
        self.create_subscription(Empty, START_TOPIC, self._on_start, 10)

        self._results = []

    def run(self):
        self._wait_for_services()
        self._wait_for_ekf_ready()
        self._wait_for_start()
        self._check_set_mode()
        self._check_arm()
        self._check_takeoff()
        self._check_hover()
        self._check_land_and_disarm()

        self._print_summary()
        self.get_logger().info('interface takeoff/land test PASSED: Pi commanded arm -> takeoff '
                               '-> hover -> land -> disarm, all confirmed via telemetry')

    # -- 遥测回调 --

    def _on_armed(self, msg):
        self._latest_armed = msg.data

    def _on_ekf(self, msg):
        self._latest_ekf = msg

    def _on_local_pose(self, msg):
        self._latest_local_pose = msg

    def _on_start(self, msg):
        self._start_triggered = True

    # -- 辅助 --

    def _relative_altitude(self):
        return -self._latest_local_pose.pose.position.z

    def _record_result(self, name, target, achieved, error):
        self._results.append((name, target, achieved, error))

    def _print_summary(self):
        self.get_logger().info('=== SUMMARY (target vs achieved vs error) ===')
        for name, target, achieved, error in self._results:
            self.get_logger().info(f'  {name}: target={target} achieved={achieved} error={error}')

    def _wait_for_condition(self, predicate, description, timeout, settle_sec=0.0):
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

    # -- 起降链路 --

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
            lambda: self._relative_altitude() >= self._takeoff_altitude - self._altitude_tolerance,
            'altitude telemetry confirming climb', self._takeoff_timeout)
        climb_err = self._relative_altitude() - self._takeoff_altitude
        self._record_result('Takeoff', f'{self._takeoff_altitude:.1f}m',
                            f'{self._relative_altitude():.2f}m', f'{climb_err:+.2f}m')
        self.get_logger().info(f'takeoff confirmed (alt={self._relative_altitude():.2f}m)')

    def _check_hover(self):
        # 悬停观察：不发任何指令，看高度能不能稳住在目标附近，记录最大偏差。
        self.get_logger().info(f'holding hover for {self._hover_hold:.0f}s, watching altitude ...')
        deadline = time.monotonic() + self._hover_hold
        max_dev = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            max_dev = max(max_dev, abs(self._relative_altitude() - self._takeoff_altitude))
        self._record_result('Hover altitude hold', f'{self._takeoff_altitude:.1f}m',
                            f'max_dev={max_dev:.2f}m', f'{max_dev:.2f}m')
        self.get_logger().info(f'hover confirmed (max altitude deviation={max_dev:.2f}m)')

    def _check_land_and_disarm(self):
        self.get_logger().info('calling set_mode(LAND) ...')
        result = self._call(self._set_mode_client, SetMode.Request(mode='LAND'), 'set_mode')
        if not result.success:
            raise RuntimeError('set_mode(LAND) returned success=False')
        self._wait_for_condition(
            lambda: self._latest_armed is False,
            'armed telemetry confirming auto-disarm after landing', self._land_timeout)
        self._record_result('Land + auto-disarm', 'armed=False', 'armed=False', 'OK')
        self.get_logger().info('landed and disarmed (confirmed via telemetry)')


def main(args=None):
    rclpy.init(args=args)
    node = InterfaceTakeoffLandTest()
    exit_code = 0
    try:
        node.run()
    except Exception as exc:  # noqa: BLE001 — 测试失败要非零退出，脚本据此判成败
        node.get_logger().error(f'interface takeoff/land test FAILED: {exc}')
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
