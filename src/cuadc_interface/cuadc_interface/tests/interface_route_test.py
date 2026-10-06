"""全任务航线测试（不用 CV，如夜间）：按正式任务的航线飞一遍，验证实机全链路（转场、高度、速度、
低空悬停、返航），不接近真筒、不投放。

和任务算法完全解耦：只通过接口层话题/服务指挥飞机，不起决策/感知，不读任务配置——算法怎么改都不用
适配本测试。放行/飞手解锁/夺控/兜底 RTL/返航降落直接继承 interface_flight_suite_test（同一套起飞
条件与夺控语义，见其文件头），本文件只换中间航段。场地数字默认按规则书（docs/spec/field-layout.md，
与 mission.real.yaml 当前值一致），现场可用参数改（如场地不够长）。

航线（场地坐标 = 起飞坐标系轴：+x 起飞时机头方向，+y 右；原点 = 起飞点）：
  起飞到 takeoff_altitude_m → 以 transit_speed 飞往投放区 →
  假投放目标逐个：目标上方 cruise_alt → 降到 stub_alt 悬停 stub_hover_sec（量低空保持误差 + 投放门式条件的满足占比，
  只统计、不投放）→ 升回 cruise_alt →
  以 transit_speed 飞往侦察区 → search_alt 按巡视航点各到一次 → RTL 返航降落 → 汇总表。

单独手动跑（或 scripts/run_route_test_real.sh / _sim.sh）：
  ros2 run cuadc_interface interface_route_test --ros-args -r __node:=interface_route_test
"""
import math
import sys
import time

import rclpy
from std_msgs.msg import Float64

from cuadc_interface_msgs.msg import PositionTarget
from cuadc_interface.tests import interface_flight_suite_test as fs


def _pairs(flat):
    """[x1, y1, x2, y2, ...] -> [(x1, y1), (x2, y2), ...]（ROS 参数只支持一维数组）。"""
    return [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]


class InterfaceRouteTest(fs.InterfaceFlightSuiteTest):

    def __init__(self):
        super().__init__()
        self.declare_parameter('cruise_alt_m', 3.0)                 # 转场/假目标上方高度
        self.declare_parameter('search_alt_m', 5.0)                 # 侦察区巡视高度
        self.declare_parameter('stub_alt_m', 1.4)                   # 假目标处下降悬停高度（= 高投放变体投放高度）
        self.declare_parameter('stub_hover_sec', 3.0)
        self.declare_parameter('stub_targets', [32.5, -1.0, 32.5, 1.0])     # 场地 (x,y) 对：投放区中心 ±1m
        self.declare_parameter('recon_waypoints', [56.3, 2.0, 56.3, -2.0, 58.7, -2.0, 58.7, 2.0])  # 侦察区中心 57.5 + 正式任务巡视偏移
        self.declare_parameter('transit_speed_ms', 5.0)
        self.declare_parameter('zone_speed_ms', 3.0)
        self.declare_parameter('hover_tolerance_m', 0.3)            # 低空悬停前到位判据（比航点 0.6m 严）
        self.declare_parameter('transit_leg_timeout_sec', 60.0)     # 长转场段（~30m）到位超时
        # 仅统计、不触发任何动作：悬停中有多少时间同时满足"像投放门"的条件（只靠飞控保持位置，不含视觉误差）
        self.declare_parameter('gate_tol_m', 0.05)
        self.declare_parameter('gate_speed_ms', 0.15)
        self.declare_parameter('gate_level_rad', 0.1)

        g = self.get_parameter
        self._cruise_alt = g('cruise_alt_m').value
        self._search_alt = g('search_alt_m').value
        self._stub_alt = g('stub_alt_m').value
        self._stub_hover = g('stub_hover_sec').value
        self._stub_targets = _pairs(list(g('stub_targets').value))
        self._recon_waypoints = _pairs(list(g('recon_waypoints').value))
        self._transit_speed = g('transit_speed_ms').value
        self._zone_speed = g('zone_speed_ms').value
        self._hover_tol = g('hover_tolerance_m').value
        self._transit_timeout = g('transit_leg_timeout_sec').value
        self._gate = (g('gate_tol_m').value, g('gate_speed_ms').value, g('gate_level_rad').value)
        self._latest_roll_pitch = None

        # 飞控重进位置控制会退回 WP_SPD，1Hz 重发保证当前段限速生效（同任务 decision_node 做法）。
        self._speed_pub = self.create_publisher(Float64, '/cuadc_interface/cmd/speed', 10)
        self._speed = None
        self.create_timer(1.0, self._resend_speed)

    def _on_attitude(self, msg):
        super()._on_attitude(msg)
        q = msg.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self._latest_roll_pitch = (roll, pitch)

    def _gate_like(self, herr):
        """本拍是否同时满足投放门式条件：到点误差 / 水平+竖直速度 / 姿态（与任务投放门同阈值）。"""
        tol, vmax, level = self._gate
        v = self._latest_velocity.twist.linear
        rp = self._latest_roll_pitch or (0.0, 0.0)
        return (herr < tol and math.hypot(v.x, v.y) < vmax and abs(v.z) < vmax
                and abs(rp[0]) < level and abs(rp[1]) < level)

    def _resend_speed(self):
        if self._speed is not None and self._pi_in_control:
            self._speed_pub.publish(Float64(data=float(self._speed)))

    def _set_speed(self, speed):
        self._speed = speed
        self._resend_speed()

    def run(self):
        self._wait_for_services()
        self._wait_for_ekf_ready()
        self._wait_for_start()
        self._wait_for_pilot_arm()
        self._take_control()
        t0 = time.monotonic()
        try:
            self._check_takeoff()
            self._fly_route()
        except fs.PilotIntervention:
            raise
        except Exception:
            self._failsafe_rtl()  # Pi 自己的步骤失败(飞手没介入)：先尽力 RTL 再退出
            raise
        self._check_rtl_and_disarm()
        self._record_result('Total route', 'takeoff -> RTL landed', f'{time.monotonic() - t0:.0f}s', '-')
        self._print_summary()
        if self._warnings:
            self.get_logger().warn(f'route test COMPLETED with {len(self._warnings)} warning(s):')
            for w in self._warnings:
                self.get_logger().warn(f'  - {w}')
        else:
            self.get_logger().info('route test PASSED')

    # -- 航线 --

    def _leg(self, x, y, alt, label, speed, timeout=None):
        """飞到场地 (x, y)、高度 alt，记录段耗时 + 到位误差（_goto_local 以起飞后 home 为原点）。"""
        self._set_speed(speed)
        saved, self._leg_timeout = self._leg_timeout, timeout or self._leg_timeout
        t = time.monotonic()
        try:
            self._goto_local(x, y, alt, label)
        finally:
            self._leg_timeout = saved
        self._record_result(f'{label} time', '-', f'{time.monotonic() - t:.1f}s', '-')

    def _hover_at(self, x, y, alt, label):
        """到 (x, y, alt) 收紧到位后悬停 stub_hover_sec，量这段时间的水平/高度偏差（低空保持精度、GPS 漂移）。"""
        tx, ty = self._home_x + x, self._home_y + y
        self.get_logger().info(f'{label}: descend to {alt:.1f}m and hover {self._stub_hover:.0f}s (no release)')
        self._position_target_local_pub.publish(PositionTarget(x=tx, y=ty, z=-alt))
        self._wait_for_condition(
            lambda: self._distance_xy(tx, ty) <= self._hover_tol
            and abs(self._relative_altitude() - alt) <= self._hover_tol,
            f'{label} reaching hover point', self._leg_timeout, settle_sec=1.0)
        horiz, vert, gate = [], [], []
        end = time.monotonic() + self._stub_hover
        while time.monotonic() < end:
            self._spin(0.05)
            horiz.append(self._distance_xy(tx, ty))
            vert.append(self._relative_altitude() - alt)
            try:
                gate.append((time.monotonic(), self._gate_like(horiz[-1])))
            except Exception:  # noqa: BLE001 — 纯统计，出错只少一项，绝不打断飞行
                pass
        rms = math.sqrt(sum(h * h for h in horiz) / len(horiz))
        self._record_result(f'{label} hover', f'alt={alt:.1f}m {self._stub_hover:.0f}s',
                            f'horiz rms={rms:.3f}m max={max(horiz):.3f}m', f'alt err max={max(map(abs, vert)):.2f}m')
        self.get_logger().info(f'{label}: hover horiz rms={rms:.3f}m max={max(horiz):.3f}m')
        if gate:
            frac = sum(ok for _, ok in gate) / len(gate)
            longest, run_start = 0.0, None
            for t, ok in gate:
                if ok and run_start is None:
                    run_start = t
                elif not ok and run_start is not None:
                    longest, run_start = max(longest, t - run_start), None
            if run_start is not None:
                longest = max(longest, gate[-1][0] - run_start)
            self._record_result(f'{label} gate-like', f'err<{self._gate[0] * 100:.0f}cm & v<{self._gate[1]} & level',
                                f'{frac * 100:.0f}% of hover', f'longest run {longest:.2f}s')

    def _fly_route(self):
        for i, (x, y) in enumerate(self._stub_targets, start=1):
            first = i == 1
            self._leg(x, y, self._cruise_alt, f'Stub target {i} above',
                      self._transit_speed if first else self._zone_speed,
                      self._transit_timeout if first else None)
            self._set_speed(self._zone_speed)
            self._hover_at(x, y, self._stub_alt, f'Stub target {i}')
            self._leg(x, y, self._cruise_alt, f'Stub target {i} climb', self._zone_speed)
        for i, (x, y) in enumerate(self._recon_waypoints, start=1):
            first = i == 1
            self._leg(x, y, self._search_alt, f'Recon wp {i}',
                      self._transit_speed if first else self._zone_speed,
                      self._transit_timeout if first else None)


def main(args=None):
    rclpy.init(args=args)
    node = InterfaceRouteTest()
    exit_code = 0
    try:
        node.run()
    except fs.PilotIntervention as exc:
        node.get_logger().warn(f'route test ABORTED — {exc}; Pi stopped commanding, pilot has control')
        exit_code = 1
    except Exception as exc:  # noqa: BLE001 — 测试失败要非零退出，脚本据此判成败
        node.get_logger().error(f'route test FAILED: {exc}')
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
