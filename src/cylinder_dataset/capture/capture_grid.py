#!/usr/bin/env python3
"""方法二：树莓派自主控制飞控（GUIDED 解锁/起飞/走立体格点/返航），
在每个格点稳定后拍一张 + 记同刻位姿。相机/位姿/配置/写盘与 capture_external.py
共用 dataset_core.py；本脚本额外负责飞行。跑法/前提/安全见 ../README.md。

自主飞行——务必户外、GPS 3D 锁定、EKF 位置可信、遥控做安全接管在手。
这是独立工具，不需要起正式任务的 ROS2 bringup（它自己独占飞控串口）。

飞控指令口径与已验证的 cuadc_interface/interface_node.py 一致：GUIDED 用
set_mode + 心跳确认、NAV_TAKEOFF、SET_POSITION_TARGET_LOCAL_NED(LOCAL_NED,
位置+绝对yaw)、CONDITION_YAW 归一化。
"""

import argparse
import math
import os
import time
from datetime import datetime, timezone

import yaml
from pymavlink import mavutil

import dataset_core as core

# 位置+绝对 yaw、忽略速度/加速度/yaw_rate（与 interface_node 同口径）。
_POS_YAW_TYPE_MASK = 0b0000101111111000


def _wait_mode(master, listener, mode_name, mode_id, timeout):
    """反复发 set_mode 直到心跳 custom_mode 确认（ArduPilot 偶尔丢首条）。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        master.set_mode(mode_id)
        time.sleep(0.5)
        hb = listener.latest('HEARTBEAT')
        if hb is not None and hb.custom_mode == mode_id:
            return True
    return False


def _send_command_ack(master, listener, command, params, timeout=5.0):
    master.mav.command_long_send(master.target_system, master.target_component,
                                 command, 0, *params)
    deadline = time.time() + timeout
    while time.time() < deadline:
        ack = listener.latest('COMMAND_ACK')
        if ack is not None and ack.command == command:
            return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        time.sleep(0.1)
    return False


def _goto_local(master, north, east, alt, yaw_rad):
    """飞到相对起飞点的 (north,east) + 高度 alt(向上为正)，锁定 yaw。"""
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, _POS_YAW_TYPE_MASK,
        north, east, -alt,           # NED：z 向下为负
        0, 0, 0, 0, 0, 0, yaw_rad, 0)


def build_grid(grid_cfg):
    """生成格点 (north, east, alt) 列表；每层内蛇形遍历，层由低到高。"""
    g = grid_cfg
    cn, ce = g['center_offset_ne_m']

    def axis(spec):
        n, s = spec['count'], spec['spacing_m']
        return [(i - (n - 1) / 2.0) * s for i in range(n)]

    norths = [cn + d for d in axis(g['north'])]
    easts = [ce + d for d in axis(g['east'])]
    points = []
    for alt in g['altitudes_m']:
        snake = list(enumerate(norths))
        for j, e in enumerate(easts):
            rows = snake if j % 2 == 0 else list(reversed(snake))
            for _, n in rows:
                points.append((n, e, alt))
    return points


def _yaw_to_cylinder(north, east, session):
    """镜头朝向圆筒地面中心的绝对 yaw(rad)。取不到圆筒位置则返回 None。"""
    cyl = (session.get('cylinder') or {}).get('ground_center_ned')
    if not cyl:
        return None
    dn, de = cyl[0] - north, cyl[1] - east
    return math.atan2(de, dn)   # NED：yaw 从北朝东


def _dist3(lp, north, east, alt):
    return math.sqrt((lp.x - north) ** 2 + (lp.y - east) ** 2
                     + (lp.z - (-alt)) ** 2)


def fly_grid(master, listener, writer, grid_cfg, session):
    flight = grid_cfg['flight']
    points = build_grid(grid_cfg['grid'])
    print(f'格点共 {len(points)} 个。')

    mode_ids = master.mode_mapping() or {}
    guided = mode_ids.get('GUIDED')
    if guided is None:
        print('ERROR: 取不到 GUIDED 模式号。')
        return

    print('切 GUIDED ...')
    if not _wait_mode(master, listener, 'GUIDED', guided, 5.0):
        print('ERROR: 切 GUIDED 失败。')
        return
    print('解锁 ...')
    if not _send_command_ack(master, listener,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                             (1, 0, 0, 0, 0, 0, 0)):
        print('ERROR: 解锁被拒（看 PreArm / GPS / 遥控）。')
        return

    ta = flight['takeoff_alt_m']
    print(f'起飞到 {ta} m ...')
    if not _send_command_ack(master, listener,
                             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                             (0, 0, 0, 0, 0, 0, ta)):
        print('ERROR: 起飞指令被拒。')
        return
    # 等爬升到接近目标高度
    deadline = time.time() + 30
    while time.time() < deadline:
        lp = listener.latest('LOCAL_POSITION_NED')
        if lp is not None and -lp.z >= ta - 0.3:
            break
        time.sleep(0.2)

    # 设定点间速度（best-effort）
    _send_command_ack(master, listener,
                      mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                      (1, flight['speed_mps'], -1, 0, 0, 0, 0))

    yaw_mode = flight.get('yaw_mode', 'face_cylinder')
    for idx, (north, east, alt) in enumerate(points):
        yaw = 0.0
        if yaw_mode == 'face_cylinder':
            y = _yaw_to_cylinder(north, east, session)
            if y is not None:
                yaw = y
            else:
                # 取不到圆筒位置时锁定当前朝向，别落回默认 WP_YAW_BEHAVIOR
                att = listener.latest('ATTITUDE')
                yaw = att.yaw if att else 0.0
        else:
            att = listener.latest('ATTITUDE')
            yaw = att.yaw if att else 0.0

        print(f'-> 点 {idx + 1}/{len(points)} '
              f'N={north:.1f} E={east:.1f} Alt={alt:.1f} yaw={math.degrees(yaw):.0f}')
        arrived = False
        t0 = time.time()
        while time.time() - t0 < flight['arrival_timeout_s']:
            _goto_local(master, north, east, alt, yaw)   # 持续重发保持目标
            time.sleep(0.5)
            lp = listener.latest('LOCAL_POSITION_NED')
            if lp is not None and _dist3(lp, north, east, alt) \
                    <= flight['arrival_radius_m']:
                arrived = True
                break
        if not arrived:
            print('   !! 到点超时，就地拍摄并标记。')

        time.sleep(flight['settle_time_s'])
        writer.capture(extra={'grid': {
            'index': idx,
            'target_ned': [round(north, 3), round(east, 3), round(-alt, 3)],
            'target_alt_m': alt,
            'target_yaw_deg': round(math.degrees(yaw), 1),
            'arrived': arrived,
        }})

    end = flight.get('end_action', 'rtl')
    print(f'采集完成，末端动作: {end}')
    if end in ('rtl', 'land'):
        target = 'RTL' if end == 'rtl' else 'LAND'
        tid = mode_ids.get(target)
        if tid is not None:
            _wait_mode(master, listener, target, tid, 5.0)
    # loiter：留在 GUIDED 悬停，等人用遥控接管


def preflight_ok(listener, timeout=20.0):
    """自主起飞前的最低门槛：轮询等待 有姿态 + GPS 3D 锁定 + EKF 位置可信。
    真机上 GPS 锁定/EKF 收敛需要时间，故等到就绪或超时，而非一次性判定。"""
    deadline = time.time() + timeout
    att = gr = ekf = None
    trust = False
    while time.time() < deadline:
        att = listener.latest('ATTITUDE')
        gr = listener.latest('GPS_RAW_INT')
        ekf = listener.latest('EKF_STATUS_REPORT')
        trust = bool(ekf and (ekf.flags & core._EKF_POS_HORIZ_ABS)
                     and (ekf.flags & core._EKF_POS_VERT_ABS))
        if att is not None and gr is not None and gr.fix_type >= 3 and trust:
            print(f'  ✓ 姿态 OK | GPS fix={gr.fix_type} sats={gr.satellites_visible}'
                  ' | EKF 位置可信')
            return True
        time.sleep(0.5)
    # 超时，报告卡在哪
    if att is None:
        print('  x 收不到 ATTITUDE')
    if gr is None or gr.fix_type < 3:
        print(f'  x GPS 未 3D 锁定 (fix={getattr(gr, "fix_type", None)})')
    if not trust:
        print('  x EKF 位置不可信（pos_horiz_abs/pos_vert_abs 未置位）')
    return False


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    core.add_common_args(ap)
    ap.add_argument('--grid', default=None,
                    help='格点飞行配置 yaml（默认脚本旁 grid.yaml）')
    ap.add_argument('--yes', action='store_true',
                    help='跳过起飞前人工确认（默认会要求确认）')
    args = ap.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    cfg, calibration = core.resolve_runtime(args, script_dir)
    grid_path = args.grid or os.path.join(script_dir, 'grid.yaml')
    with open(grid_path) as f:
        grid_cfg = yaml.safe_load(f)
    with open(args.session) as f:
        session = yaml.safe_load(f)
    session_name = session.get('session') or 'session'

    master = core.connect_fc(cfg)
    listener = core.PoseListener(master)
    listener.start()
    core.request_streams(master, cfg['rate'])

    print('初始化相机 ...')
    backend = core.make_backend(cfg)
    print(f'  相机后端: {backend.kind}')

    print('\n起飞前检查:')
    if not preflight_ok(listener):
        print('preflight 未通过，退出（不解锁）。')
        listener.stop(); backend.close(); return

    home = core.capture_home_snapshot(master, listener)
    n_pts = len(build_grid(grid_cfg['grid']))
    print(f'\n即将自主飞行 {n_pts} 个格点，末端 '
          f'{grid_cfg["flight"].get("end_action", "rtl")}。'
          f'确认遥控在手可随时接管。')
    if not args.yes:
        if input('输入 GO 回车开始自主飞行（其它=取消）: ').strip() != 'GO':
            print('已取消，不解锁。')
            listener.stop(); backend.close(); return

    writer = core.DatasetWriter(cfg['out'], session_name, backend, listener)
    writer.write_run_meta({
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'method': 'autonomous_grid_flight',
        'connection': f"{cfg['connection']}@{cfg['baud']}",
        'resolution': [cfg['width'], cfg['height']],
        'camera_backend': backend.kind,
        'image_format': cfg['image_format'], 'jpeg_quality': cfg['jpeg_quality'],
        'home_position': home,
        'grid_cfg': grid_cfg,
        'camera_calibration': calibration,
        'session_meta': session,
    })
    print(f'\n输出目录: {writer.out_dir}\n')

    try:
        fly_grid(master, listener, writer, grid_cfg, session)
    except KeyboardInterrupt:
        print('\n(Ctrl-C) 中止——请立即用遥控接管！')
    finally:
        writer.close()
        backend.close()
        listener.stop()
        print(f'\n结束，共 {writer.frame_no} 张，输出在 {writer.out_dir}')


if __name__ == '__main__':
    main()
