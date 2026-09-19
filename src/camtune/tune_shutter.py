#!/usr/bin/env python3
"""camtune/tune_shutter.py —— 台架相机快门/增益/对焦扫描(IMX519 + AK7375)。

调试工具(非任务件):独占相机、用 rpicam-still 以**手动快门/增益/对焦**
(关 AE、关 AF)扫一格网格,每档拍几帧存盘,并算 **锐度(Laplacian 方差)+
亮度** 供排序。运动模糊的**条纹长度**最好在存下的图上用肉眼/看图工具量
(节拍器标记),脚本给的锐度只是辅助排序(注意锐度会被亮度混淆——暗帧锐度
天然低,要在亮度相近时比)。方法与依据见 docs/spec/camera-tuning.md。

在 Pi 上跑。相机是独占资源,先停图传底座、跑完再起:
    sudo systemctl stop  cuadc-onboard-camera
    python3 tune_shutter.py --shutters 500,1000,2000,4000 --gains 1,4,8 --lens-position 1.0
    sudo systemctl start cuadc-onboard-camera
"""
from __future__ import annotations

import argparse
import csv
import os
import shutil
import subprocess
import sys
from datetime import datetime

import cv2


def _require_rpicam() -> str:
    for exe in ("rpicam-still", "libcamera-still"):
        path = shutil.which(exe)
        if path:
            return path
    sys.exit("找不到 rpicam-still/libcamera-still,确认在树莓派上运行。")


def capture(exe: str, path: str, shutter_us: int, gain: float,
            lens_pos: float, width: int, height: int) -> None:
    """拍一张:手动快门+增益(=关 AE)、手动对焦(=关 AF)、关降噪(别掩盖噪声/模糊)。"""
    cmd = [
        exe, "-n", "--immediate", "-t", "500",
        "--shutter", str(shutter_us),          # µs;1/2000s≈500µs
        "--gain", f"{gain:.2f}",               # 模拟增益(给了 shutter+gain 即手动曝光)
        "--awbgains", "1.8,1.8",               # 固定白平衡,让各档亮度可比
        "--autofocus-mode", "manual",
        "--lens-position", f"{lens_pos:.2f}",  # 屈光度:0=∞,1/距离(m)≈对该距离
        "--denoise", "cdn_off",                # 不让降噪抹掉噪声/模糊
        "--width", str(width), "--height", str(height),
        "-o", path,
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        err = (r.stderr or r.stdout or "").strip()
        if any(k in err.lower() for k in ("busy", "device or resource", "failed to acquire", "camera")):
            sys.exit("相机被占用?先停图传底座:sudo systemctl stop cuadc-onboard-camera\n" + err)
        raise RuntimeError(err or "rpicam-still failed")


def frame_metrics(path: str) -> tuple[float, float] | None:
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return None
    sharp = float(cv2.Laplacian(img, cv2.CV_64F).var())  # 越大越锐(受亮度混淆)
    bright = float(img.mean())
    return sharp, bright


def _nums(s: str, typ):
    return [typ(x) for x in s.split(",") if x.strip()]


def main() -> None:
    ap = argparse.ArgumentParser(description="IMX519 台架快门/增益/对焦扫描")
    ap.add_argument("--shutters", default="500,1000,2000,4000,8000",
                    help="逗号分隔曝光时间(µs);1/2000s≈500µs、1/1000s≈1000µs")
    ap.add_argument("--gains", default="1,4,8", help="逗号分隔模拟增益")
    ap.add_argument("--lens-position", type=float, default=0.0,
                    help="对焦屈光度(0=∞;对 1m 用 1.0、0.5m 用 2.0)——先对准目标距离")
    ap.add_argument("--frames", type=int, default=3,
                    help="每档拍几帧(运动相位不同,取最糊那张看)")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--out", default="~/camtune_out")
    args = ap.parse_args()

    exe = _require_rpicam()
    session = os.path.join(os.path.expanduser(args.out),
                           datetime.now().strftime("sweep_%Y%m%d_%H%M%S"))
    os.makedirs(session, exist_ok=True)
    shutters = _nums(args.shutters, int)
    gains = _nums(args.gains, float)
    print(f"输出: {session}")
    print(f"扫描: shutters(µs)={shutters}  gains={gains}  lens={args.lens_position}  "
          f"每档 {args.frames} 帧\n")

    rows: list[tuple] = []
    agg: dict[tuple[int, float], list[float]] = {}
    for g in gains:
        for sh in shutters:
            sharps = []
            for i in range(args.frames):
                name = f"sh{sh}_g{g:g}_{i}.jpg"
                path = os.path.join(session, name)
                try:
                    capture(exe, path, sh, g, args.lens_position, args.width, args.height)
                except RuntimeError as e:
                    print(f"[FAIL] {name}: {e}", file=sys.stderr)
                    continue
                m = frame_metrics(path)
                if m is None:
                    continue
                sharp, bright = m
                sharps.append(sharp)
                rows.append((sh, g, i, round(sharp, 1), round(bright, 1), name))
            if sharps:
                agg[(sh, g)] = sharps
                print(f"shutter={sh:5d}µs gain={g:<4g} "
                      f"sharp[min/mean]={min(sharps):8.1f}/{sum(sharps)/len(sharps):8.1f}")

    with open(os.path.join(session, "index.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["shutter_us", "gain", "frame", "lap_var", "brightness", "file"])
        w.writerows(rows)

    print(f"\n存了 {len(rows)} 帧 + index.csv 于 {session}")
    print("怎么看:①肉眼看节拍器标记的条纹——越短越好(运动模糊的直接量);")
    print("       ②lap_var 越大越锐,但只在亮度相近时比(暗帧锐度天然低);")
    print("       ③亮度太低=该档进光不够,要么升增益(噪声)、要么加光/放长快门。")


if __name__ == "__main__":
    main()
