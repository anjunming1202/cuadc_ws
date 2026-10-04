"""接口层起降测试（最小 smoke）：飞手 RC 解锁后，树莓派经接口层接管飞控完成
「切 GUIDED → 起飞到 H → 悬停观察 → 降落 → 自动上锁」这一条最短链路。

现场飞行的第一步就跑这个：先确认 Pi 真能指挥飞控起降、遥测能如实报回，
再上 interface_flight_suite_test 那套全向/速度/高度的完整合集。不做任何
水平移动，飞机只在起飞点正上方上下——风险最低。飞手 RC 全程待命。

起飞条件（固定顺序）：放行信号 → 放行后看到一次「未解锁→解锁」（飞手 RC 解锁）→
解锁时模式是 STABILIZE → Pi 立刻切 GUIDED 起飞。Pi 不发 arm：解锁/上锁归飞手，
RC arm 开关挡位始终和飞机状态一致，任何时刻拨一下就停桨。sim 没有飞手，
`-p emulate_pilot_arm:=true`（仅 sim 脚本传）由节点代飞手切 STABILIZE+解锁。

飞手夺控：Pi 接管后若模式变成 Pi 没设过的（拨开关/飞控 failsafe）或在空中被上锁，
立即停发任何指令、不再切模式，打 ABORTED 退出；只有 Pi 自己的步骤失败才兜底切 LAND。

高度 = local_position 的 `-z`：接口层已换算成相对起飞点（解锁点），见 04-interfaces.md。
sim/real 同一节点，连接目标由各自的启动脚本写死（run_flight_check_sim/real.sh）。

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
from std_msgs.msg import Bool, Empty, String

# 放行信号话题（飞行测试节点共用），同最终任务的 task_start_trigger 套路。
START_TOPIC = '/cuadc_flight_test/start'
_ARM_MODE = 'STABILIZE'                  # 飞手解锁时必须在的模式（开关不在此挡会顶掉 GUIDED）
_IN_AIR_DISARM_ALT_M = 1.0               # 上锁时离起飞点高于此 = 不是正常落地


class PilotIntervention(Exception):
    """Pi 接管后飞手/飞控改了模式或上锁：Pi 必须立即停手，不能再发任何指令。"""


class InterfaceTakeoffLandTest(Node):

    def __init__(self):
        super().__init__('interface_takeoff_land_test')
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('ekf_ready_timeout_sec', 60.0)
        self.declare_parameter('ekf_settle_sec', 5.0)  # healthy 须持续此秒数再放行(抗瞬态)
        # 放行信号：默认开(现场安全)。sim 快速迭代可 -p wait_for_start:=false。
        self.declare_parameter('wait_for_start', True)
        self.declare_parameter('start_timeout_sec', 600.0)
        self.declare_parameter('pilot_arm_timeout_sec', 120.0)
        self.declare_parameter('emulate_pilot_arm', False)  # 仅 sim：节点代飞手解锁
        self.declare_parameter('takeoff_altitude_m', 1.0)
        self.declare_parameter('takeoff_timeout_sec', 30.0)
        self.declare_parameter('altitude_tolerance_m', 0.5)
        self.declare_parameter('hover_hold_sec', 8.0)
        self.declare_parameter('land_timeout_sec', 60.0)
        g = self.get_parameter
        self._service_timeout = g('service_timeout_sec').value
        self._ekf_ready_timeout = g('ekf_ready_timeout_sec').value
        self._ekf_settle_sec = g('ekf_settle_sec').value
        self._wait_for_start_enabled = g('wait_for_start').value
        self._start_timeout = g('start_timeout_sec').value
        self._pilot_arm_timeout = g('pilot_arm_timeout_sec').value
        self._emulate_pilot_arm = g('emulate_pilot_arm').value
        self._takeoff_altitude = g('takeoff_altitude_m').value
        self._takeoff_timeout = g('takeoff_timeout_sec').value
        self._altitude_tolerance = g('altitude_tolerance_m').value
        self._hover_hold = g('hover_hold_sec').value
        self._land_timeout = g('land_timeout_sec').value

        self._arm_client = self.create_client(Arm, '/cuadc_interface/arm')
        self._set_mode_client = self.create_client(SetMode, '/cuadc_interface/set_mode')
        self._takeoff_client = self.create_client(Takeoff, '/cuadc_interface/takeoff')

        self._latest_armed = None
        self._latest_mode = None
        self._latest_ekf = None
        self._latest_local_pose = None
        self._start_triggered = False
        self.create_subscription(Bool, '/cuadc_interface/status/armed', self._on_armed, 10)
        self.create_subscription(String, '/cuadc_interface/status/mode', self._on_mode, 10)
        self.create_subscription(
            EkfStatus, '/cuadc_interface/telemetry/ekf_status', self._on_ekf, 10)
        self.create_subscription(
            PoseStamped, '/cuadc_interface/telemetry/local_position', self._on_local_pose, 10)
        self.create_subscription(Empty, START_TOPIC, self._on_start, 10)

        self._results = []
        # 接管/夺控状态：Pi 切 GUIDED 并确认后才算接管；接管后只允许 Pi 自己设过的模式。
        self._pi_in_control = False
        self._allowed_modes = set()
        self._expect_disarm = False      # Pi 发了 LAND 后的上锁是预期的
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
            self._check_hover()
        except PilotIntervention:
            raise
        except Exception:
            # Pi 自己的步骤失败(飞手没介入)：不能直接退出让飞机在 GUIDED 里无人指挥，先尽力 LAND。
            self._failsafe('LAND')
            raise
        self._check_land_and_disarm()

        self._print_summary()
        self.get_logger().info('interface takeoff/land test PASSED: pilot armed -> Pi took off '
                               '-> hover -> land -> disarm, all confirmed via telemetry')

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

    def _spin(self, timeout_sec):
        # 所有等待都经这里：每处理一次回调就查一次夺控，飞手一介入立刻中断当前步骤。
        rclpy.spin_once(self, timeout_sec=timeout_sec)
        if self._intervention:
            raise PilotIntervention(self._intervention)

    def _spin_for(self, duration_sec):
        end = time.monotonic() + duration_sec
        while time.monotonic() < end:
            self._spin(0.1)

    def _wait_for_condition(self, predicate, description, timeout, settle_sec=0.0):
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
            f'    ros2 topic pub -1 {START_TOPIC} std_msgs/msg/Empty "{{}}"')
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

    # -- 起降链路 --

    def _check_takeoff(self):
        self._wait_for_condition(
            lambda: self._latest_local_pose is not None, 'local_position samples', 10.0)
        self.get_logger().info(f'calling takeoff(alt={self._takeoff_altitude}) ...')
        result = self._call(
            self._takeoff_client, Takeoff.Request(alt=self._takeoff_altitude), 'takeoff')
        if not result.success:
            raise RuntimeError('takeoff returned success=False')
        self._wait_for_condition(
            lambda: self._relative_altitude() >= self._takeoff_altitude * 0.8,
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
            self._spin(0.2)
            max_dev = max(max_dev, abs(self._relative_altitude() - self._takeoff_altitude))
        self._record_result('Hover altitude hold', f'{self._takeoff_altitude:.1f}m',
                            f'max_dev={max_dev:.2f}m', f'{max_dev:.2f}m')
        self.get_logger().info(f'hover confirmed (max altitude deviation={max_dev:.2f}m)')

    def _check_land_and_disarm(self):
        self.get_logger().info('calling set_mode(LAND) ...')
        self._expect_disarm = True
        self._set_mode('LAND')
        self._wait_for_condition(
            lambda: self._latest_armed is False,
            'armed telemetry confirming auto-disarm after landing', self._land_timeout)
        if self._disarm_alt is not None and self._disarm_alt > _IN_AIR_DISARM_ALT_M:
            raise RuntimeError(f'disarmed at {self._disarm_alt:.1f}m above takeoff point — '
                               'not a landing (pilot RC kill?)')
        self._record_result('Land + auto-disarm', 'armed=False', 'armed=False', 'OK')
        self.get_logger().info('landed and disarmed (confirmed via telemetry)')

    def _failsafe(self, mode):
        if self._intervention:
            return
        self.get_logger().error(f'flight step failed while airborne -> commanding {mode} before exit')
        try:
            self._set_mode(mode)
        except Exception as exc:  # noqa: BLE001 — 兜底本身失败只能告警,靠飞手 RC
            self.get_logger().error(f'failsafe {mode} failed ({exc}) -- PILOT TAKE OVER')


def main(args=None):
    rclpy.init(args=args)
    node = InterfaceTakeoffLandTest()
    exit_code = 0
    try:
        node.run()
    except PilotIntervention as exc:
        node.get_logger().warn(f'interface takeoff/land test ABORTED — {exc}; '
                               'Pi stopped commanding, pilot has control')
        exit_code = 1
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
