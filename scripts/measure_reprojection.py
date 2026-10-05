#!/usr/bin/env python3
"""sim-only 反投影真值量测（ADR-0010 D 的"尺子"，一键入口 run_measure_reprojection.sh）。
逐筒悬停 nadir/侧偏 + 1m/s 动态飞越，采样 /perception/detections 与 field/config.py
真值比对，报 bias(配置错则大)/std(噪声/时序错则大)。判定门只加 nadir(|bias|<0.05,
std<0.03)，退出码 0/1 可当回归门；各视点角色与实测数见 sim-stress-testbed §3.3。
"""
import math
import sys
import time
from pathlib import Path

import rclpy
from pymavlink import mavutil
from rclpy.node import Node

from cuadc_perception_msgs.msg import DetectionArray

# 筒真值单一出处：field/config.py（未安装成包，按仓库相对路径引入）
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src' / 'cuadc_sim' / 'field'))
import config as field_config  # noqa: E402

_ALT_M = 2.0                 # 量测悬停高度 = 任务巡航高度
_OFFSET_E_M = 0.55           # 侧偏视点的东向偏移(锁 yaw=0 后筒落在水平半FOV内)
_SETTLE_S = 5.0              # 到点后先稳定再采样
_SAMPLE_S = 6.0              # 每视点采样时长（10Hz 检测 ≈ 60 帧）
_SPAWN_E, _SPAWN_N = field_config.DROP_ZONE_BENCH_SPAWN  # 仿真世界 x->NED east, y->NED north
_ASSOC_GATE_M = 0.8          # 检测←→真值筒关联门（筒间距≥1.5m，不会串）
_BIAS_LIMIT_M = 0.05         # 干净 sim 判定门限（只加在 nadir 视点上,见下）
_STD_LIMIT_M = 0.03
# PASS 门只看 nadir：它检验的是配置自洽(偏置/内参/挂载/平面高)。斜视(offset)视点
# 只作信息输出——空心白筒斜视时近侧外壁可见,色块质心向观察者偏移,反投影随之偏
# (真机同样存在的物理特性,不是配置错;对 SEARCH 记忆选点有参考价值)。


def _tubes_ned():
    """真值筒表 -> [(label, north, east, radius_m)]：仿真世界 -> 飞控 NED（世界 x = 东、y = 北）。
    本工具直接用飞控 NED 飞行和比对；感知检测在起飞坐标系，见 _Collector._cb 的换算。"""
    out = []
    for label, dia, _h in field_config.DROP_CYLINDERS:
        wx, wy = field_config.DROP_CYLINDER_POSITIONS[label]
        out.append((label, wy, wx, dia / 2))
    return out


class _Collector(Node):
    def __init__(self):
        super().__init__('measure_reprojection')
        self.samples = []            # (north, east, apparent_size)
        self.enabled = False
        self.yaw0 = None             # 解锁时航向(飞控 NED)，= 接口层冻结的起飞坐标系 +x
        self.create_subscription(DetectionArray, '/perception/detections', self._cb, 10)

    def _cb(self, msg):
        if not self.enabled:
            return
        for d in msg.detections:
            if d.target_type == 'cylinder':
                # 检测在起飞坐标系（原点 = 解锁点 = bench 出生点，x = 解锁时机头），本工具用飞控 NED
                # 飞行/比对：按解锁航向转回 NED 再加回出生点（见 docs/spec/coordinate-frames.md）。
                c, s = math.cos(self.yaw0), math.sin(self.yaw0)
                x, y = d.position.x, d.position.y
                self.samples.append((x * c - y * s + _SPAWN_N, x * s + y * c + _SPAWN_E,
                                     d.apparent_size))


def _goto(mav, north, east, wait_s):
    # type_mask 用位置 + yaw(=0 朝北)：锁死机头朝向 -> 图像朝向确定(图像上=北)，
    # 否则 GUIDED 自动朝航向偏航,窄 FOV 下筒会掉出画面。
    t0 = time.monotonic()
    while time.monotonic() - t0 < wait_s:
        mav.mav.set_position_target_local_ned_send(
            0, mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000101111111000,
            north, east, -_ALT_M, 0, 0, 0, 0, 0, 0, 0.0, 0)
        time.sleep(0.5)


def _sample_at(node, mav, north, east):
    _goto(mav, north, east, _SETTLE_S)
    node.samples = []
    node.enabled = True
    t0 = time.monotonic()
    while time.monotonic() - t0 < _SAMPLE_S:
        _goto(mav, north, east, 0.4)   # 持续发点保持位置
        rclpy.spin_once(node, timeout_sec=0.05)
    node.enabled = False
    return list(node.samples)


def _sample_moving(node, mav, tn, te):
    """受控 1m/s 向东飞越筒上方采样：位姿-图像时刻差的误差 ∝ v×时刻差(沿东)，静态
    显不出来；bias_e 可直接换算等效时刻差。本底/证据链数值见 sim-stress-testbed §3.3。"""
    _goto(mav, tn, te - 1.5, _SETTLE_S + 2.0)
    node.samples = []
    node.enabled = True
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.2:
        # 速度模式: vx=0, vy(东)=1.0, vz=0, yaw=0 锁北 (mask: pos/acc/yawrate 忽略)
        mav.mav.set_position_target_local_ned_send(
            0, mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000100111000111,
            0, 0, 0, 0.0, 1.0, 0.0, 0, 0, 0, 0.0, 0)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)
    node.enabled = False
    return list(node.samples)


def _stats(samples, tn, te):
    """关联到该筒的样本 -> (n, bias_n, bias_e, std)。"""
    xs = [(n, e) for n, e, _ in samples if math.hypot(n - tn, e - te) < _ASSOC_GATE_M]
    if not xs:
        return 0, float('nan'), float('nan'), float('nan')
    mn = sum(n for n, _ in xs) / len(xs)
    me = sum(e for _, e in xs) / len(xs)
    var = sum((n - mn) ** 2 + (e - me) ** 2 for n, e in xs) / len(xs)
    return len(xs), mn - tn, me - te, math.sqrt(var)


def main():
    rclpy.init()
    node = _Collector()
    mav = mavutil.mavlink_connection('udp:127.0.0.1:14551')
    print('waiting heartbeat...')
    mav.wait_heartbeat()
    time.sleep(10)  # EKF 就绪
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0)
    time.sleep(2)
    armed = False
    for _ in range(6):
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        armed = bool(hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if armed:
            break
        time.sleep(2)
    if not armed:
        print('FAIL: arm rejected')
        return 1
    att = mav.recv_match(type='ATTITUDE', blocking=True, timeout=5)
    node.yaw0 = att.yaw
    print(f'takeoff heading (frame +x) = {math.degrees(att.yaw):.1f}deg')
    mav.mav.command_long_send(mav.target_system, mav.target_component,
                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, _ALT_M)
    time.sleep(10)

    rows, ok = [], True
    for label, tn, te, radius in _tubes_ned():
        for view in ('nadir', 'offset', 'moving'):
            if view == 'nadir':
                samples = _sample_at(node, mav, tn, te)
            elif view == 'offset':
                samples = _sample_at(node, mav, tn, te + _OFFSET_E_M)
            else:
                samples = _sample_moving(node, mav, tn, te)
            cnt, bn, be, std = _stats(samples, tn, te)
            bias = math.hypot(bn, be) if cnt else float('nan')
            gated = view == 'nadir'  # 判定门只加 nadir，offset 是信息输出(见文件头)
            good = cnt > 0 and bias < _BIAS_LIMIT_M and std < _STD_LIMIT_M
            if gated:
                ok &= good
            rows.append((label, view, cnt, bn, be, bias, std, good, gated))
            print(f'tube {label} [{view:6s}] n={cnt:3d} bias=({bn:+.3f},{be:+.3f}) '
                  f'|bias|={bias:.3f} std={std:.3f}  {"OK" if good else "info" if not gated else "BAD"}')

    print('\n===== reprojection vs ground truth =====')
    print(f'{"tube":4s} {"view":6s} {"n":>3s} {"bias_n":>7s} {"bias_e":>7s} {"|bias|":>7s} {"std":>6s}')
    for label, view, cnt, bn, be, bias, std, good, gated in rows:
        mark = ('OK' if good else 'BAD') if gated else 'info'
        print(f'{label:4s} {view:6s} {cnt:3d} {bn:+7.3f} {be:+7.3f} {bias:7.3f} {std:6.3f} {mark}')
    print(f'RESULT: {"PASS" if ok else "FAIL"} '
          f'(nadir gate: |bias|<{_BIAS_LIMIT_M} std<{_STD_LIMIT_M}; offset rows informational)')
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
