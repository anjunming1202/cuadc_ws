#!/usr/bin/env python3
"""圆筒数据集采集的共享核心：相机后端、飞控位姿监听、运行配置合并、
按运行分文件夹写盘（图像 + JSON 边车 + CSV 索引 + run_meta.json）。

两种采集方法共用本模块，只在"谁控制飞控"上不同：
- capture_external.py：飞控由外部控制（手持/遥控/Mission Planner），本机只读+拍照。
- capture_grid.py    ：树莓派自主飞行走立体格点，边飞边拍。

不接入任务链路（parallel 测试模块，同 dronelink）。
"""

import csv
import json
import math
import os
import subprocess
import threading
import time
from datetime import datetime, timezone

import yaml
from pymavlink import mavutil

# EKF_STATUS_REPORT.flags 里判定水平/垂直位置估计"绝对可信"的两个位，口径
# 与接口层 cuadc_interface/interface_node.py 的 _EKF_POS_READY 一致。
_EKF_POS_HORIZ_ABS = mavutil.mavlink.EKF_POS_HORIZ_ABS
_EKF_POS_VERT_ABS = mavutil.mavlink.EKF_POS_VERT_ABS

# 必须是 list：pymavlink recv_match(type=...) 只特判 list，传 tuple 会被包成
# [(...)] 而永不匹配、永远返回 None。HEARTBEAT/COMMAND_ACK 供自主飞行确认
# 模式切换 / 指令 ack。
_WATCHED = ['ATTITUDE', 'LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT',
            'GPS_RAW_INT', 'EKF_STATUS_REPORT', 'HOME_POSITION',
            'HEARTBEAT', 'COMMAND_ACK']

_GPS_FIX_NAMES = {0: 'no_gps', 1: 'no_fix', 2: '2d', 3: '3d',
                  4: 'dgps', 5: 'rtk_float', 6: 'rtk_fixed'}


# --------------------------------------------------------------------------
# 飞控位姿监听（唯一读者线程；命令发送由主线程做，与读互不冲突）
# --------------------------------------------------------------------------
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

    def latest(self, msg_type):
        with self._lock:
            hit = self._latest.get(msg_type)
        return hit[1] if hit else None

    def wait_for(self, msg_type, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.latest(msg_type)
            if msg is not None:
                return msg
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
            'roll_deg': _rad2deg(m.roll), 'pitch_deg': _rad2deg(m.pitch),
            'yaw_deg': _rad2deg(m.yaw),
            'quaternion_wxyz': _euler_to_quat(m.roll, m.pitch, m.yaw),
            'msg_age_s': age(t),
        }
    lp = snap.get('LOCAL_POSITION_NED')
    if lp:
        t, m = lp
        rec['local_position_ned'] = {
            'x': m.x, 'y': m.y, 'z': m.z, 'vx': m.vx, 'vy': m.vy, 'vz': m.vz,
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
        _, m = ekf
        rec['ekf_status'] = {
            'flags': int(m.flags),
            'pos_horiz_abs': bool(m.flags & _EKF_POS_HORIZ_ABS),
            'pos_vert_abs': bool(m.flags & _EKF_POS_VERT_ABS),
        }
    rec['position_trustworthy'] = bool(
        ekf and (ekf[1].flags & _EKF_POS_HORIZ_ABS)
        and (ekf[1].flags & _EKF_POS_VERT_ABS))
    return rec


# --------------------------------------------------------------------------
# 相机后端：picamera2(常驻) 优先，回退 rpicam-still(每张 spawn)
# --------------------------------------------------------------------------
class CameraBackend:

    def __init__(self, prefer, width, height, image_format='jpg', quality=95):
        self.width, self.height = width, height
        self.image_format = image_format   # 'jpg' | 'png'
        self.quality = quality
        self.kind = None
        self._picam = None
        if prefer in ('auto', 'picamera2'):
            try:
                from picamera2 import Picamera2
                self._picam = Picamera2()
                cfg = self._picam.create_still_configuration(
                    main={'size': (width, height)})
                self._picam.configure(cfg)
                self._picam.options['quality'] = quality
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
            self._picam.capture_file(path)   # 格式由 path 后缀决定
            return True, t
        cmd = ['rpicam-still', '-n', '--immediate', '-t', '200',
               '--width', str(self.width), '--height', str(self.height),
               '--encoding', self.image_format, '-q', str(self.quality),
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


# --------------------------------------------------------------------------
# 运行配置合并：内置默认 < capture.yaml < 命令行
# --------------------------------------------------------------------------
RUNTIME_DEFAULTS = {
    'out': '~/cylinder_dataset_out',
    'connection': '/dev/ttyAMA0',
    'baud': 57600,
    'width': 1920,
    'height': 1080,
    'rate': 15,
    'backend': 'auto',
    'image_format': 'jpg',
    'jpeg_quality': 95,
}


def add_common_args(ap):
    """两个采集脚本共享的命令行参数。"""
    ap.add_argument('--session', required=True, help='会话 yaml（物理测量）路径')
    ap.add_argument('--config', default=None,
                    help='运行配置 yaml（默认脚本旁 capture.yaml，若存在）')
    ap.add_argument('--out', default=None, help='数据集输出根目录')
    ap.add_argument('--connection', default=None, help='飞控连接串')
    ap.add_argument('--baud', type=int, default=None)
    ap.add_argument('--width', type=int, default=None)
    ap.add_argument('--height', type=int, default=None)
    ap.add_argument('--rate', type=int, default=None,
                    help='请求遥测的数据流频率 Hz（越高位姿越新鲜）')
    ap.add_argument('--backend', choices=['auto', 'picamera2', 'rpicam'],
                    default=None, help='相机后端')
    ap.add_argument('--format', choices=['jpg', 'png'], default=None,
                    help='图像格式；png 无损但大很多')
    ap.add_argument('--quality', type=int, default=None,
                    help='JPEG 质量 1-100（png 忽略）')


def resolve_runtime(args, script_dir):
    """合并运行参数并返回 (cfg, camera_calibration)。"""
    cfg = dict(RUNTIME_DEFAULTS)
    config_path = args.config or os.path.join(script_dir, 'capture.yaml')
    file_cfg = {}
    if os.path.exists(config_path):
        with open(config_path) as f:
            file_cfg = yaml.safe_load(f) or {}
        print(f'加载运行配置: {config_path}')
    for k in RUNTIME_DEFAULTS:
        if file_cfg.get(k) is not None:
            cfg[k] = file_cfg[k]
    cli = {'out': args.out, 'connection': args.connection, 'baud': args.baud,
           'width': args.width, 'height': args.height, 'rate': args.rate,
           'backend': args.backend, 'image_format': args.format,
           'jpeg_quality': args.quality}
    for k, v in cli.items():
        if v is not None:
            cfg[k] = v
    return cfg, file_cfg.get('camera_calibration')


def make_backend(cfg):
    prefer = ('picamera2' if cfg['backend'] == 'picamera2'
              else ('rpicam-still' if cfg['backend'] == 'rpicam' else 'auto'))
    return CameraBackend(prefer, cfg['width'], cfg['height'],
                         cfg['image_format'], cfg['jpeg_quality'])


def connect_fc(cfg):
    print(f"连接飞控 {cfg['connection']} @ {cfg['baud']} ...")
    master = mavutil.mavlink_connection(cfg['connection'], baud=cfg['baud'])
    master.wait_heartbeat()
    print(f'已连上 system {master.target_system} '
          f'component {master.target_component}')
    return master


def request_streams(master, rate):
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)


def capture_home_snapshot(master, listener, passive=False):
    """取一次 home/EKF 原点，用于把 local NED 关联到经纬度。passive 时不发查询。"""
    if not passive:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, 0)
    hp = listener.wait_for('HOME_POSITION', timeout=3.0)
    if hp is None:
        return None
    return {'lat_deg': hp.latitude / 1e7, 'lon_deg': hp.longitude / 1e7,
            'alt_m': hp.altitude / 1e3}


# --------------------------------------------------------------------------
# 按运行分文件夹写盘
# --------------------------------------------------------------------------
class DatasetWriter:
    """一次运行 = 一个带时间戳的新文件夹。写图像 + JSON 边车 + index.csv。"""

    def __init__(self, out_root, session_name, backend, listener):
        self.backend = backend
        self.listener = listener
        run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_id = run_id
        self.out_dir = os.path.join(os.path.expanduser(out_root),
                                    f'{session_name}_{run_id}')
        os.makedirs(os.path.join(self.out_dir, 'images'), exist_ok=True)
        self.session_name = session_name
        self.frame_no = 0
        self._csv_f = open(os.path.join(self.out_dir, 'index.csv'), 'w',
                           newline='')
        self._writer = csv.writer(self._csv_f)
        self._writer.writerow(
            ['frame', 'image', 'capture_time_utc', 'roll_deg', 'pitch_deg',
             'yaw_deg', 'ned_x', 'ned_y', 'ned_z', 'lat_deg', 'lon_deg',
             'rel_alt_m', 'gps_fix', 'sats', 'position_trustworthy',
             'attitude_age_s'])

    def write_run_meta(self, meta):
        meta = dict(meta)
        meta['run_id'] = self.run_id
        with open(os.path.join(self.out_dir, 'run_meta.json'), 'w') as mf:
            json.dump(meta, mf, indent=2, ensure_ascii=False)

    def capture(self, extra=None):
        """拍一张 + 抓同刻位姿 + 写 JSON/CSV。extra 合并进 record。返回 rec|None。"""
        no = self.frame_no + 1
        name = f'frame_{no:04d}'
        img_rel = os.path.join('images', f'{name}.{self.backend.image_format}')
        img_path = os.path.join(self.out_dir, img_rel)

        ok, t_capture = self.backend.capture(img_path)
        if not ok:
            return None
        snap = self.listener.snapshot()
        if 'ATTITUDE' not in snap:
            os.remove(img_path)
            print('  !! 还没收到 ATTITUDE，飞控在飞行状态吗？丢弃这张。')
            return None

        pose = build_pose_record(snap, t_capture)
        record = {
            'frame': no, 'image': img_rel, 'session': self.session_name,
            'capture_time_utc': datetime.fromtimestamp(
                t_capture, timezone.utc).isoformat(),
            'capture_monotonic_s': round(t_capture, 3),
            'resolution': [self.backend.width, self.backend.height],
            'pose': pose,
        }
        if extra:
            record.update(extra)
        with open(os.path.join(self.out_dir, name + '.json'), 'w') as jf:
            json.dump(record, jf, indent=2, ensure_ascii=False)

        att = pose.get('attitude', {})
        lp = pose.get('local_position_ned', {})
        gp = pose.get('global_position', {})
        gps = pose.get('gps', {})
        self._writer.writerow(
            [no, img_rel, record['capture_time_utc'],
             round(att.get('roll_deg', 0.0), 2),
             round(att.get('pitch_deg', 0.0), 2),
             round(att.get('yaw_deg', 0.0), 2),
             lp.get('x'), lp.get('y'), lp.get('z'),
             gp.get('lat_deg'), gp.get('lon_deg'), gp.get('relative_alt_m'),
             gps.get('fix'), gps.get('satellites_visible'),
             pose.get('position_trustworthy'), att.get('msg_age_s')])
        self._csv_f.flush()
        self.frame_no = no
        self._print_line(record)
        return record

    @staticmethod
    def _print_line(rec):
        pose = rec['pose']
        att = pose.get('attitude', {})
        gps = pose.get('gps', {})
        trust = 'OK' if pose.get('position_trustworthy') else '不可信'
        print(f"  [{rec['frame']}] {rec['image']} | "
              f"RPY=({att.get('roll_deg', 0):.1f},{att.get('pitch_deg', 0):.1f},"
              f"{att.get('yaw_deg', 0):.1f}) | "
              f"GPS={gps.get('fix', '?')}/{gps.get('satellites_visible', '?')}星 | "
              f"位置{trust} | 位姿龄{att.get('msg_age_s', '?')}s")

    def close(self):
        self._csv_f.close()
