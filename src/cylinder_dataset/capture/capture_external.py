#!/usr/bin/env python3
"""方法一：飞控由外部控制（手持 / 遥控 Stabilize / Mission Planner），
本机只从飞控读数据 + 拍照，不向飞控发任何控制指令。

在树莓派上跑。手动逐张（回车触发）或定时连拍（--interval）。每次运行单独
一个带时间戳的新文件夹。相机/位姿/配置/写盘逻辑见 dataset_core.py，与自主
飞行脚本 capture_grid.py 共用。跑法/前提见 ../README.md。

只发遥测请求（数据流 + 一次性查 home），不是控制指令，不会 override 遥控
Stabilize 或 Mission Planner AUTO。要字节级只读加 --passive（需飞控已在推流）。
"""

import argparse
import os
import sys
import time
from datetime import datetime, timezone

import yaml

import dataset_core as core


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    core.add_common_args(ap)
    ap.add_argument('--interval', type=float, default=None,
                    help='定时连拍间隔秒；不给则手动逐张（回车触发）')
    ap.add_argument('--count', type=int, default=None,
                    help='连拍模式下最多拍多少张；不给则拍到 Ctrl-C')
    ap.add_argument('--passive', action='store_true',
                    help='字节级只读：不请求数据流、不查 home（需飞控已在推流）')
    args = ap.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    cfg, calibration = core.resolve_runtime(args, script_dir)
    with open(args.session) as f:
        session = yaml.safe_load(f)
    session_name = session.get('session') or 'session'

    master = core.connect_fc(cfg)
    listener = core.PoseListener(master)
    listener.start()
    if not args.passive:
        core.request_streams(master, cfg['rate'])

    print('初始化相机 ...')
    backend = core.make_backend(cfg)
    print(f'  相机后端: {backend.kind}')
    if calibration and calibration.get('resolution') not in (
            None, [cfg['width'], cfg['height']]):
        print(f"  !! 警告：拍摄分辨率 {[cfg['width'], cfg['height']]} 与标定 "
              f"{calibration.get('resolution')} 不一致，畸变矫正会不准。")

    home = core.capture_home_snapshot(master, listener, passive=args.passive)

    writer = core.DatasetWriter(cfg['out'], session_name, backend, listener)
    writer.write_run_meta({
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'method': 'external_control',
        'connection': f"{cfg['connection']}@{cfg['baud']}",
        'resolution': [cfg['width'], cfg['height']],
        'camera_backend': backend.kind,
        'image_format': cfg['image_format'], 'jpeg_quality': cfg['jpeg_quality'],
        'passive_readonly': args.passive,
        'mode': 'interval' if args.interval else 'manual',
        'interval_s': args.interval,
        'home_position': home,
        'camera_calibration': calibration,
        'session_meta': session,
    })
    if home is None:
        print('  (未取到 HOME_POSITION；室外等 GPS 3D 锁定后再采效果更好)')
    print(f'\n输出目录: {writer.out_dir}')

    try:
        if args.interval:
            print(f'定时连拍，每 {args.interval}s 一张，Ctrl-C 结束。\n')
            while args.count is None or writer.frame_no < args.count:
                loop_start = time.time()
                writer.capture()
                sleep = args.interval - (time.time() - loop_start)
                if sleep > 0:
                    time.sleep(sleep)
        else:
            print('手动模式：回车=拍一张，q + 回车=结束。')
            print('提示：用 rpicam 后端时需先 '
                  '`sudo systemctl stop dronelink-video` 释放相机。\n')
            while True:
                line = sys.stdin.readline()
                if not line or line.strip().lower() == 'q':
                    break
                writer.capture()
    except KeyboardInterrupt:
        print('\n(Ctrl-C)')
    finally:
        writer.close()
        backend.close()
        listener.stop()
        print(f'\n结束，共 {writer.frame_no} 张，输出在 {writer.out_dir}')


if __name__ == '__main__':
    main()
