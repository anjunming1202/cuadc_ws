"""接口层 ARM + RC 接管观察（**每次飞行测试前必做**，也是比赛前给裁判演示
「RC 能夺控/disarm」的必要环节）。验证/观察两件事：

  1. 树莓派能否 arm —— 最小的「Pi→飞控」控制链路打通（能 arm 就说明指令、
     链路、PreArm 基本都通，能不能飞的第一道门）。
  2. RC 能否 disarm / 接管 —— Pi arm 后进入观察窗口，安全飞手用 RC 执行 disarm
     （杀锁）或切模式夺控；节点**如实记录**看到的事件（armed 掉、mode 变、failsafe）。

**本节点只如实观察+汇报，不判 PASS/FAIL**：每种情况打一条 log，RC 介入时给正向提示。
armed→false 到底是 RC 杀锁还是飞控自动 disarm，节点按时间戳如实报，由现场判读（飞手
最清楚自己有没有拨杀锁）。退出码只在**根本没法跑**（连不上/EKF 超时/arm 被拒）时非零。

流程：放行闸门(桨不转) → set_mode+arm → 确认 armed → 观察窗口(默认 15s，每 1s 打倒计时)：
  - 窗口内 armed→false：记录时间，结束（RC 杀锁 或 接近飞控 DISARM_DELAY 的自动 disarm）；
  - mode 离开起飞模式(仍 armed)：记「RC 成功接管」，继续观察；
  - failsafe active：记「failsafe 触发」；
  - 窗口结束仍 armed：等飞控自带 DISARM_DELAY 自动 disarm（宽限观察）；宽限后仍 armed →
    WARN 并 Pi 兜底 disarm（纯安全网，正常不会走到）。

飞控自动 disarm 是 ArduCopter 自带 `DISARM_DELAY`（默认约 10s、可配）——armed 后地面
无起飞状态到期自动上锁，本节点不自己定超时，超时出口交给它；窗口设 ≥ DISARM_DELAY。

安全：arm 会让电机转(有桨则桨转)。地面做本测试按队内 SOP 决定是否拆桨——只验 RC 信号
链路用拆桨最安全。飞手 RC 全程在手。`mode` 默认 GUIDED；无 GPS 室内可 `-p mode:=STABILIZE`。

不在 launch 里，由 run_arm_check_{sim,real}.sh 起（自带接口层 + 放行提示）。
"""
import sys
import time

import rclpy
from rclpy.node import Node

from cuadc_interface_msgs.msg import EkfStatus, FailsafeState
from cuadc_interface_msgs.srv import Arm, SetMode
from std_msgs.msg import Bool, Empty, String

# arm 放行闸门话题（所有会 arm 的测试共用），同最终任务 task_start_trigger 套路。
START_TOPIC = '/cuadc_flight_test/start'


class InterfaceArmTest(Node):

    def __init__(self):
        super().__init__('interface_arm_test')
        self.declare_parameter('service_timeout_sec', 10.0)
        self.declare_parameter('ekf_ready_timeout_sec', 60.0)
        self.declare_parameter('wait_for_start', True)
        self.declare_parameter('start_timeout_sec', 600.0)
        self.declare_parameter('mode', 'GUIDED')
        self.declare_parameter('observe_window_sec', 15.0)     # 观察窗口；≥ 飞控 DISARM_DELAY
        self.declare_parameter('countdown_tick_sec', 1.0)      # 倒计时打印间隔
        self.declare_parameter('post_window_grace_sec', 8.0)   # 窗口后等飞控自动 disarm 的宽限
        self.declare_parameter('disarm_confirm_timeout_sec', 15.0)
        g = self.get_parameter
        self._service_timeout = g('service_timeout_sec').value
        self._ekf_ready_timeout = g('ekf_ready_timeout_sec').value
        self._wait_for_start_enabled = g('wait_for_start').value
        self._start_timeout = g('start_timeout_sec').value
        self._mode = g('mode').value
        self._observe_window = g('observe_window_sec').value
        self._tick_sec = g('countdown_tick_sec').value
        self._post_window_grace = g('post_window_grace_sec').value
        self._disarm_confirm_timeout = g('disarm_confirm_timeout_sec').value

        self._arm_client = self.create_client(Arm, '/cuadc_interface/arm')
        self._set_mode_client = self.create_client(SetMode, '/cuadc_interface/set_mode')

        self._latest_armed = None
        self._latest_mode = None
        self._latest_ekf = None
        self._latest_failsafe = None
        self._start_triggered = False
        self.create_subscription(Bool, '/cuadc_interface/status/armed', self._on_armed, 10)
        self.create_subscription(String, '/cuadc_interface/status/mode', self._on_mode, 10)
        self.create_subscription(
            EkfStatus, '/cuadc_interface/telemetry/ekf_status', self._on_ekf, 10)
        self.create_subscription(
            FailsafeState, '/cuadc_interface/status/failsafe', self._on_failsafe, 10)
        self.create_subscription(Empty, START_TOPIC, self._on_start, 10)

        self._events = []   # (elapsed_or_None, text) 事件时间线

    # -- 回调 --

    def _on_armed(self, msg):
        self._latest_armed = msg.data

    def _on_mode(self, msg):
        self._latest_mode = msg.data

    def _on_ekf(self, msg):
        self._latest_ekf = msg

    def _on_failsafe(self, msg):
        self._latest_failsafe = msg

    def _on_start(self, msg):
        self._start_triggered = True

    # -- 辅助 --

    def _failsafe_active(self):
        return self._latest_failsafe is not None and self._latest_failsafe.active

    def _log_event(self, elapsed, text):
        self._events.append((elapsed, text))

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
        for name, client in (('arm', self._arm_client), ('set_mode', self._set_mode_client)):
            if not client.wait_for_service(timeout_sec=self._service_timeout):
                raise RuntimeError(
                    f'{name} service not available -- is cuadc_interface running and connected?')

    def _wait_for_ekf_ready(self):
        self._wait_for_condition(
            lambda: self._latest_ekf is not None and self._latest_ekf.healthy,
            'EKF/GPS becoming healthy', self._ekf_ready_timeout)
        self.get_logger().info('EKF/GPS ready')

    def _wait_for_start(self):
        if not self._wait_for_start_enabled:
            return
        self.get_logger().warn(
            'READY — EKF ok, NOT armed. ARM 已挡住。确认现场安全(按 SOP 决定是否拆桨)、'
            'RC 在手后,从另一个 SSH 发放行:\n'
            f'    ros2 topic pub -1 {START_TOPIC} std_msgs/msg/Empty "{{}}"')
        self._wait_for_condition(
            lambda: self._start_triggered, 'arm-test start trigger', self._start_timeout)

    # -- 主流程 --

    def run(self):
        self._wait_for_services()
        self._wait_for_ekf_ready()
        self._wait_for_start()

        # 1) Pi arm（最小控制打通）
        result = self._call(self._set_mode_client, SetMode.Request(mode=self._mode), 'set_mode')
        if not result.success:
            raise RuntimeError(f'set_mode({self._mode}) returned success=False')
        result = self._call(self._arm_client, Arm.Request(arm=True), 'arm')
        if not result.success:
            raise RuntimeError(f'Pi 无法 arm(飞控拒绝): {result.message} -- 最小控制链未打通,不要飞')
        self._wait_for_condition(
            lambda: self._latest_armed is True, 'armed telemetry confirming True', 10.0)
        self.get_logger().warn(
            f'*** ARMED ***(电机已转,有桨则桨转)。观察窗口 {int(self._observe_window)}s 开始：'
            '安全飞手现在可用 RC disarm(杀锁)或切模式夺控;不动则到期由飞控自带 DISARM_DELAY 自动 disarm。')
        self._log_event(0.0, f'Pi arm 成功(armed=true, mode={self._latest_mode})')

        # 2) 观察窗口
        start_mode = self._latest_mode
        t0 = time.monotonic()
        tick = -1
        override_logged = False
        failsafe_logged = False
        disarm_elapsed = None
        while True:
            rclpy.spin_once(self, timeout_sec=0.2)
            elapsed = time.monotonic() - t0
            if elapsed >= self._observe_window:
                break
            cur_tick = int(elapsed / self._tick_sec)
            if cur_tick > tick:
                tick = cur_tick
                remaining = max(0, int(self._observe_window - elapsed))
                self.get_logger().info(
                    f'[T+{elapsed:4.1f}s/{int(self._observe_window)}s 剩{remaining}s] '
                    f'armed={self._latest_armed} mode={self._latest_mode}')
            if self._latest_armed is False:
                disarm_elapsed = elapsed
                self.get_logger().warn(
                    f'检测到 armed→false @T+{elapsed:.1f}s → 若此刻你拨了 RC 杀锁＝'
                    'RC 成功 disarm 介入;未动 RC 则为飞控自动 disarm(按现场判读)')
                self._log_event(elapsed, 'armed→false')
                break
            if start_mode is not None and self._latest_mode not in (None, start_mode) \
                    and not override_logged:
                override_logged = True
                self.get_logger().warn(
                    f'检测到 RC 成功接管：mode {start_mode}→{self._latest_mode} '
                    f'@T+{elapsed:.1f}s(控制权已交飞手,继续观察)')
                self._log_event(elapsed, f'RC 接管 mode {start_mode}→{self._latest_mode}')
            if self._failsafe_active() and not failsafe_logged:
                failsafe_logged = True
                reason = getattr(self._latest_failsafe, 'reason', '')
                self.get_logger().warn(f'检测到 failsafe 触发({reason}) @T+{elapsed:.1f}s')
                self._log_event(elapsed, f'failsafe {reason}')

        # 3) 出口
        if disarm_elapsed is None:
            self.get_logger().info(
                f'观察窗口 {int(self._observe_window)}s 结束仍 armed → 等飞控自动 disarm(DISARM_DELAY)'
                f',最多再看 {int(self._post_window_grace)}s ...')
            tg0 = time.monotonic()
            while time.monotonic() - tg0 < self._post_window_grace:
                rclpy.spin_once(self, timeout_sec=0.2)
                if self._latest_armed is False:
                    gt = time.monotonic() - tg0
                    self.get_logger().warn(f'飞控自动 disarm @窗口后+{gt:.1f}s')
                    self._log_event(self._observe_window + gt, '飞控自动 disarm')
                    disarm_elapsed = self._observe_window + gt
                    break
            if disarm_elapsed is None:
                self.get_logger().error(
                    '飞控未在宽限内自动 disarm(检查 DISARM_DELAY 是否被禁用/过大) → Pi 兜底 disarm')
                result = self._call(self._arm_client, Arm.Request(arm=False), 'disarm')
                if not result.success:
                    raise RuntimeError(f'Pi 兜底 disarm 失败: {result.message}')
                self._wait_for_condition(
                    lambda: self._latest_armed is False, 'armed telemetry confirming False',
                    self._disarm_confirm_timeout)
                self._log_event(None, 'Pi 兜底 disarm(飞控没自动 disarm)')

        self._print_timeline(override_logged, failsafe_logged, disarm_elapsed)

    def _print_timeline(self, override_logged, failsafe_logged, disarm_elapsed):
        self.get_logger().info('=== 观察时间线（如实记录，不判 PASS/FAIL）===')
        for elapsed, text in self._events:
            stamp = 'T+--  ' if elapsed is None else f'T+{elapsed:4.1f}s'
            self.get_logger().info(f'  {stamp}: {text}')
        # 中性结论
        if override_logged:
            concl = 'RC 成功接管(切模式)已观察到 — RC 夺控有效'
        elif disarm_elapsed is not None and disarm_elapsed < self._observe_window:
            concl = (f'窗口内 armed→false @T+{disarm_elapsed:.1f}s '
                     '— 你拨了 RC 杀锁则 RC disarm 有效;否则为飞控自动 disarm')
        elif disarm_elapsed is not None:
            concl = '未见 RC 介入,飞控自动 disarm 收场(RC 未演示,如需验证请重跑并用 RC 杀锁)'
        else:
            concl = '飞控未自动 disarm,已 Pi 兜底(异常,请查 DISARM_DELAY)'
        self.get_logger().info(f'本次结论：Pi arm 成功；{concl}')
        self.get_logger().info(
            '（本测试只如实汇报。RC 能否 disarm/接管请对照上面事件与你的 RC 操作确认。）')


def main(args=None):
    rclpy.init(args=args)
    node = InterfaceArmTest()
    exit_code = 0
    try:
        node.run()
    except Exception as exc:  # noqa: BLE001 — 跑不起来(连不上/EKF超时/arm被拒)才非零退出
        node.get_logger().error(f'interface arm test 无法完成: {exc}')
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
