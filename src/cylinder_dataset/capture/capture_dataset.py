#!/usr/bin/env python3
"""在树莓派上采集「圆筒图像 ↔ 同一时刻飞控位置+姿态」配对数据集（室外 GPS）。

跑法/前提见 ../README.md。要点：
- 每张照片抓一份飞控位姿快照，并记录每个位姿字段相对拍照时刻的新鲜度
  (msg_age_s)，让"同一时刻对应"可量化、可核验。
- 室外 GPS：运行开始记录 home/EKF 原点 + GPS 质量；local_position_ned 即
  相对 home 的 NED 偏移，配合 EKF 可信位判断位置是否可用。
- 两种连拍：手动逐张(默认)或定时连拍(--interval)。每次运行单独一个带
  时间戳的新文件夹，互不覆盖。
- 相机后端：优先 picamera2(常驻相机，抓帧延迟低、同刻更紧)，未装则回退
  rpicam-still(每张 spawn 一次)。

不接入任务链路，是并列的离线数据采集工具。
"""

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

import yaml
from pymavlink import mavutil

# EKF_STATUS_REPORT.flags 里判定水平/垂直位置估计"绝对可信"的两个位，口径
# 与接口层 cuadc_interface/interface_node.py 的 _EKF_POS_READY 一致。
_EKF_POS_HORIZ_ABS = mavutil.mavlink.EKF_POS_HORIZ_ABS
_EKF_POS_VERT_ABS = mavutil.mavlink.EKF_POS_VERT_ABS

# 后台线程缓存这些遥测的最新一帧，供拍照时快照。
_WATCHED = ('ATTITUDE', 'LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT',
            'GPS_RAW_INT', 'EKF_STATUS_REPORT', 'HOME_POSITION')

_GPS_FIX_NAMES = {0: 'no_gps', 1: 'no_fix', 2: '2d', 3: '3d',
                  4: 'dgps', 5: 'rtk_float', 6: 'rtk_fixed'}


class PoseListener:
    """后台线程持续读串口，缓存每类消息的最新一帧 (recv_time, msg)。"""

    def __init__(self, master):
        self._master = master
        self._lock = threading.Lock()
        self._latest = {}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _loop(self):
        while not self._stop.is_set():
            msg = self._master.recv_match(type=_WATCHED, blocking=True,
                                          timeout=1.0)
            if msg is None:
                continue
            with self._lock:
                self._latest[msg.get_type()] = (time.time(), msg)

    def snapshot(self):
        with self._lock:
            return dict(self._latest)

    def wait_for(self, msg_type, timeout):
        """阻塞等某类消息出现（已缓存则立即返回其 msg），超时返回 None。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                hit = self._latest.get(msg_type)
            if hit:
                return hit[1]
            time.sleep(0.05)
        return None


def _rad2deg(x):
    return x * 180.0 / math.pi


def _euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [
        cr * cp * cy + sr * sp * sy,   # w
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
    ]


def build_pose_record(snap, ref_time):
    """把各类最新消息整理成位姿记录；msg_age_s 相对拍照时刻 ref_time。"""
    rec = {}

    def age(t):
        return round(ref_time - t, 3)

    att = snap.get('ATTITUDE')
    if att:
        t, m = att
        rec['attitude'] = {
            'roll_deg': _rad2deg(m.roll),
            'pitch_deg': _rad2deg(m.pitch),
            'yaw_deg': _rad2deg(m.yaw),
            'quaternion_wxyz': _euler_to_quat(m.roll, m.pitch, m.yaw),
            'msg_age_s': age(t),
        }

    lp = snap.get('LOCAL_POSITION_NED')
    if lp:
        t, m = lp
        rec['local_position_ned'] = {
            'x': m.x, 'y': m.y, 'z': m.z,
            'vx': m.vx, 'vy': m.vy, 'vz': m.vz,
            'msg_age_s': age(t),
        }

    gp = snap.get('GLOBAL_POSITION_INT')
    if gp:
        t, m = gp
        rec['global_position'] = {
            'lat_deg': m.lat / 1e7, 'lon_deg': m.lon / 1e7,
            'alt_m': m.alt / 1e3, 'relative_alt_m': m.relative_alt / 1e3,
            'msg_age_s': age(t),
        }

    gr = snap.get('GPS_RAW_INT')
    if gr:
        t, m = gr
        rec['gps'] = {
            'fix_type': int(m.fix_type),
            'fix': _GPS_FIX_NAMES.get(int(m.fix_type), str(m.fix_type)),
            'satellites_visible': int(m.satellites_visible),
            'hdop': m.eph / 100.0 if m.eph != 65535 else None,
            'msg_age_s': age(t),
        }

    ekf = snap.get('EKF_STATUS_REPORT')
    if ekf:
        t, m = ekf
        rec['ekf_status'] = {
            'flags': int(m.flags),
            'pos_horiz_abs': bool(m.flags & _EKF_POS_HORIZ_ABS),
            'pos_vert_abs': bool(m.flags & _EKF_POS_VERT_ABS),
        }
    # 位置是否可信：两个绝对位标记都置位才算数。
    rec['position_trustworthy'] = bool(
        ekf and (ekf[1].flags & _EKF_POS_HORIZ_ABS)
        and (ekf[1].flags & _EKF_POS_VERT_ABS))
    return rec


class CameraBackend:
    """相机抽象：picamera2(常驻) 优先，回退 rpicam-still(每张 spawn)。"""

    def __init__(self, prefer, width, height):
        self.width, self.height = width, height
        self.kind = None
        self._picam = None
        if prefer in ('auto', 'picamera2'):
            try:
                from picamera2 import Picamera2
                self._picam = Picamera2()
                cfg = self._picam.create_still_configuration(
                    main={'size': (width, height)})
                self._picam.configure(cfg)
                self._picam.start()
                time.sleep(1.0)   # 让 AE/AWB 收敛
                self.kind = 'picamera2'
            except Exception as e:
                if prefer == 'picamera2':
                    raise
                print(f'  (picamera2 不可用，回退 rpicam-still: {e})')
        if self.kind is None:
            self.kind = 'rpicam-still'

    def capture(self, path):
        """拍一张存到 path，返回 (ok, capture_time)。capture_time 尽量贴近曝光。"""
        if self.kind == 'picamera2':
            t = time.time()
            self._picam.capture_file(path)
            return True, t
        cmd = ['rpicam-still', '-n', '--immediate', '-t', '200',
               '--width', str(self.width), '--height', str(self.height),
               '-o', path]
        try:
            subprocess.run(cmd, check=True, capture_output=True)
            return True, time.time()   # 曝光在子进程末尾，返回后取时刻最贴近
        except FileNotFoundError:
            print('ERROR: 找不到 rpicam-still，确认在树莓派上运行。')
            return False, None
        except subprocess.CalledProcessError as e:
            print(f'ERROR: rpicam-still 失败: '
                  f'{e.stderr.decode(errors="replace")}')
            return False, None

    def close(self):
        if self._picam is not None:
            self._picam.stop()
            self._picam.close()


def capture_home_snapshot(master, listener):
    """运行开始时取一次 home/EKF 原点，用于把 local NED 关联到经纬度。"""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, 0)
    hp = listener.wait_for('HOME_POSITION', timeout=3.0)
    if hp is None:
        return None
    return {'lat_deg': hp.latitude / 1e7, 'lon_deg': hp.longitude / 1e7,
            'alt_m': hp.altitude / 1e3}


def write_frame(out_dir, frame_no, backend, listener, session_name):
    """拍一张 + 抓位姿快照 + 写 JSON。返回 (record | None)。"""
    name = f'frame_{frame_no:04d}'
    img_rel = os.path.join('images', name + '.jpg')
    img_path = os.path.join(out_dir, img_rel)

    ok, t_capture = backend.capture(img_path)
    if not ok:
        return None
    # 曝光时刻即刻抓位姿，msg_age_s 相对 t_capture 计算。
    snap = listener.snapshot()
    if 'ATTITUDE' not in snap:
        os.remove(img_path)
        print('  !! 还没收到 ATTITUDE，飞控在飞行状态吗？丢弃这张。')
        return None

    pose = build_pose_record(snap, t_capture)
    record = {
        'frame': frame_no,
        'image': img_rel,
        'session': session_name,
        'capture_time_utc': datetime.fromtimestamp(
            t_capture, timezone.utc).isoformat(),
        'capture_monotonic_s': round(t_capture, 3),
        'resolution': [backend.width, backend.height],
        'pose': pose,
    }
    with open(os.path.join(out_dir, name + '.json'), 'w') as jf:
        json.dump(record, jf, indent=2, ensure_ascii=False)
    return record


def print_frame_line(rec):
    pose = rec['pose']
    att = pose.get('attitude', {})
    gps = pose.get('gps', {})
    trust = 'OK' if pose.get('position_trustworthy') else '不可信'
    print(f"  [{rec['frame']}] {rec['image']} | "
          f"RPY=({att.get('roll_deg', 0):.1f},{att.get('pitch_deg', 0):.1f},"
          f"{att.get('yaw_deg', 0):.1f}) | "
          f"GPS={gps.get('fix', '?')}/{gps.get('satellites_visible', '?')}星 | "
          f"位置{trust} | 位姿龄{att.get('msg_age_s', '?')}s")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--session', required=True, help='会话 yaml 配置路径')
    ap.add_argument('--out', default='~/cylinder_dataset_out',
                    help='数据集输出根目录（默认 ~/cylinder_dataset_out）')
    ap.add_argument('--connection', default='/dev/ttyAMA0',
                    help='飞控连接串（默认 /dev/ttyAMA0）')
    ap.add_argument('--baud', type=int, default=57600)
    ap.add_argument('--width', type=int, default=1920)
    ap.add_argument('--height', type=int, default=1080)
    ap.add_argument('--rate', type=int, default=15,
                    help='请求遥测的数据流频率 Hz（越高位姿越新鲜）')
    ap.add_argument('--backend', choices=['auto', 'picamera2', 'rpicam'],
                    default='auto', help='相机后端')
    ap.add_argument('--interval', type=float, default=None,
                    help='定时连拍间隔秒；不给则手动逐张（回车触发）')
    ap.add_argument('--count', type=int, default=None,
                    help='连拍模式下最多拍多少张；不给则拍到 Ctrl-C')
    args = ap.parse_args()

    with open(args.session) as f:
        session = yaml.safe_load(f)
    session_name = session.get('session') or 'session'

    # 每次运行单独一个带时间戳的新文件夹，互不覆盖。
    run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(os.path.expanduser(args.out),
                           f'{session_name}_{run_id}')
    img_dir = os.path.join(out_dir, 'images')
    os.makedirs(img_dir, exist_ok=True)
    index_path = os.path.join(out_dir, 'index.csv')

    print(f'连接飞控 {args.connection} @ {args.baud} ...')
    master = mavutil.mavlink_connection(args.connection, baud=args.baud)
    master.wait_heartbeat()
    print(f'已连上 system {master.target_system} '
          f'component {master.target_component}')
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)

    listener = PoseListener(master)
    listener.start()

    print('初始化相机 ...')
    backend = CameraBackend(
        'picamera2' if args.backend == 'picamera2'
        else ('rpicam-still' if args.backend == 'rpicam' else 'auto'),
        args.width, args.height)
    print(f'  相机后端: {backend.kind}')

    home = capture_home_snapshot(master, listener)

    # 运行级元数据只写一份。
    run_meta = {
        'run_id': run_id,
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'connection': f'{args.connection}@{args.baud}',
        'resolution': [args.width, args.height],
        'camera_backend': backend.kind,
        'mode': 'interval' if args.interval else 'manual',
        'interval_s': args.interval,
        'home_position': home,
        'session_meta': session,
    }
    with open(os.path.join(out_dir, 'run_meta.json'), 'w') as mf:
        json.dump(run_meta, mf, indent=2, ensure_ascii=False)

    csv_f = open(index_path, 'w', newline='')
    writer = csv.writer(csv_f)
    writer.writerow(['frame', 'image', 'capture_time_utc', 'roll_deg',
                     'pitch_deg', 'yaw_deg', 'lat_deg', 'lon_deg',
                     'rel_alt_m', 'gps_fix', 'sats', 'position_trustworthy',
                     'attitude_age_s'])

    def log(rec):
        pose = rec['pose']
        att = pose.get('attitude', {})
        gp = pose.get('global_position', {})
        gps = pose.get('gps', {})
        writer.writerow([rec['frame'], rec['image'], rec['capture_time_utc'],
                         round(att.get('roll_deg', 0.0), 2),
                         round(att.get('pitch_deg', 0.0), 2),
                         round(att.get('yaw_deg', 0.0), 2),
                         gp.get('lat_deg'), gp.get('lon_deg'),
                         gp.get('relative_alt_m'), gps.get('fix'),
                         gps.get('satellites_visible'),
                         pose.get('position_trustworthy'),
                         att.get('msg_age_s')])
        csv_f.flush()
        print_frame_line(rec)

    if home is None:
        print('  (未取到 HOME_POSITION；室外等 GPS 3D 锁定后再采效果更好)')
    print(f'\n输出目录: {out_dir}')

    frame_no = 0
    try:
        if args.interval:
            print(f'定时连拍，每 {args.interval}s 一张，Ctrl-C 结束。\n')
            while args.count is None or frame_no < args.count:
                loop_start = time.time()
                rec = write_frame(out_dir, frame_no + 1, backend, listener,
                                  session_name)
                if rec is not None:
                    frame_no += 1
                    log(rec)
                sleep = args.interval - (time.time() - loop_start)
                if sleep > 0:
                    time.sleep(sleep)
        else:
            print('手动模式：回车=拍一张，q + 回车=结束。')
            print('提示：picamera2 未装时用 rpicam-still，需先 '
                  '`sudo systemctl stop dronelink-video` 释放相机。\n')
            while True:
                line = sys.stdin.readline()
                if not line or line.strip().lower() == 'q':
                    break
                rec = write_frame(out_dir, frame_no + 1, backend, listener,
                                  session_name)
                if rec is not None:
                    frame_no += 1
                    log(rec)
    except KeyboardInterrupt:
        print('\n(Ctrl-C)')
    finally:
        csv_f.close()
        backend.close()
        listener.stop()
        print(f'\n结束，共 {frame_no} 张，输出在 {out_dir}')


if __name__ == '__main__':
    main()
