#!/usr/bin/env python3
"""camtune/tune_shutter.py —— 台架相机快门/增益/对焦扫描(IMX519 + AK7375)。

调试工具(非任务件):独占相机、用 rpicam-still 以**手动快门/增益/对焦**
(关 AE、关 AF)扫一格网格,每档拍几帧存盘,并算 **锐度(Laplacian 方差)+
亮度** 供排序。运动模糊的**条纹长度**最好在存下的图上用肉眼/看图工具量
(节拍器标记),脚本给的锐度只是辅助排序(注意锐度会被亮度混淆——暗帧锐度
天然低,要在亮度相近时比)。方法与依据见 docs/spec/camera-tuning.md。

两种后端,同一套扫描/指标/CSV:
- **控制器模式(推荐 `--via-service`)**:不占相机。相机由常驻 camera-service 持有、
  仍在推地面(profile=ground-monitor 时);本脚本往控制口 5610 发参数、从本地帧端点
  5602 读帧存盘。→ **边扫边在地面 viewer 实时看,同时本地出数据**。
      python3 tune_shutter.py --via-service --shutters 500,1000,2000,4000 --gains 1,4,8 --lens-positions 1.0
- **独占模式(离线兜底)**:用 rpicam-still 自己占相机(须先停 camera-service)。
      sudo systemctl stop cuadc-onboard-camera
      python3 tune_shutter.py --shutters 500,1000,2000,4000 --gains 1,4,8 --lens-positions 1.0
      sudo systemctl start cuadc-onboard-camera
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import socket
import subprocess
import sys
import time
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


def metrics_frame(frame) -> tuple[float, float]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    bright = float(gray.mean())
    return sharp, bright


_CTRL_PIPE = (
    'udpsrc port={port} caps="application/x-rtp,media=video,encoding-name=H264,payload=96" '
    "! rtpjitterbuffer latency=50 ! rtph264depay ! h264parse ! openh264dec "
    "! videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false max-buffers=2"
)


class ServiceBackend:
    """控制器模式:往相机服务控制口发参数 + 从本地帧端点读帧(相机由服务持有、仍推地面)。"""

    def __init__(self, frame_port: int, ctrl_host: str, ctrl_port: int) -> None:
        self.cap = cv2.VideoCapture(_CTRL_PIPE.format(port=frame_port), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            sys.exit(f"cv2 打不开本地帧端点 {frame_port}；相机服务在跑吗？"
                     "(需 camera-service active,可 profile=ground-monitor)")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctrl = (ctrl_host, ctrl_port)

    def set_controls(self, shutter_us: int, gain: float, lens: float) -> None:
        cmd = {"ae": False, "exposure_us": int(shutter_us), "gain": float(gain),
               "af": "manual", "lens": float(lens)}
        self.sock.sendto(json.dumps(cmd).encode(), self.ctrl)

    def settle(self, n: int = 18) -> None:
        for _ in range(n):  # 丢掉参数生效前缓冲的旧帧
            self.cap.read()

    def grab(self):
        ok, f = self.cap.read()
        return f if ok else None

    def restore_auto(self) -> None:
        self.sock.sendto(json.dumps({"ae": True}).encode(), self.ctrl)

    def close(self) -> None:
        self.cap.release()


def _nums(s: str, typ):
    return [typ(x) for x in s.split(",") if x.strip()]


def main() -> None:
    ap = argparse.ArgumentParser(description="IMX519 台架快门/增益/对焦扫描")
    ap.add_argument("--lens-positions", default="0",
                    help="逗号分隔对焦屈光度扫描;单值=固定焦。0=∞、1m→1.0、0.5m→2.0")
    ap.add_argument("--shutters", default="500,1000,2000,4000,8000",
                    help="逗号分隔曝光时间(µs);1/2000s≈500µs、1/1000s≈1000µs")
    ap.add_argument("--gains", default="1,4,8", help="逗号分隔模拟增益")
    ap.add_argument("--distance", type=float, default=None,
                    help="本次标靶已知距离(m),仅作标签,用于建『距离→对焦』表")
    ap.add_argument("--frames", type=int, default=3,
                    help="每档拍几帧(运动相位不同,取最糊那张看)")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--out", default="~/camtune_out")
    ap.add_argument("--via-service", action="store_true",
                    help="控制器模式:不占相机,往控制口发参数 + 从本地帧端点读帧"
                         "(相机服务须在跑;图传不断,可边扫边在 viewer 看)")
    ap.add_argument("--control-port", type=int, default=5610)
    ap.add_argument("--frame-port", type=int, default=5602)
    args = ap.parse_args()

    backend = ServiceBackend(args.frame_port, "127.0.0.1", args.control_port) \
        if args.via_service else None
    exe = None if args.via_service else _require_rpicam()
    session = os.path.join(os.path.expanduser(args.out),
                           datetime.now().strftime("sweep_%Y%m%d_%H%M%S"))
    os.makedirs(session, exist_ok=True)
    lens_positions = _nums(args.lens_positions, float)
    shutters = _nums(args.shutters, int)
    gains = _nums(args.gains, float)
    dist = "" if args.distance is None else f"{args.distance:g}"
    mode = "控制器(via-service)" if backend else "独占(rpicam-still)"
    print(f"输出: {session}  模式: {mode}")
    print(f"扫描: lens={lens_positions}  shutters(µs)={shutters}  gains={gains}  "
          f"distance={dist or '-'}m  每档 {args.frames} 帧\n")

    rows: list[tuple] = []
    for lp in lens_positions:
        for g in gains:
            for sh in shutters:
                if backend:
                    backend.set_controls(sh, g, lp)
                    time.sleep(0.4)   # 让参数生效
                    backend.settle()  # 丢旧帧
                sharps = []
                for i in range(args.frames):
                    name = f"lp{lp:g}_sh{sh}_g{g:g}_{i}.jpg"
                    path = os.path.join(session, name)
                    if backend:
                        frame = backend.grab()
                        if frame is None:
                            print(f"[FAIL] {name}: no frame", file=sys.stderr)
                            continue
                        cv2.imwrite(path, frame)
                        sharp, bright = metrics_frame(frame)
                    else:
                        try:
                            capture(exe, path, sh, g, lp, args.width, args.height)
                        except RuntimeError as e:
                            print(f"[FAIL] {name}: {e}", file=sys.stderr)
                            continue
                        m = frame_metrics(path)
                        if m is None:
                            continue
                        sharp, bright = m
                    sharps.append(sharp)
                    rows.append((dist, lp, sh, g, i,
                                 round(sharp, 1), round(bright, 1), name))
                if sharps:
                    print(f"lens={lp:<4g} shutter={sh:5d}µs gain={g:<4g} "
                          f"sharp[min/mean]={min(sharps):8.1f}/{sum(sharps)/len(sharps):8.1f}")

    if backend:
        backend.restore_auto()  # 扫完恢复自动曝光,别让 viewer 停在最后一档(可能全黑)
        backend.close()

    with open(os.path.join(session, "index.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["distance_m", "lens_pos", "shutter_us", "gain", "frame",
                    "lap_var", "brightness", "file"])
        w.writerows(rows)

    print(f"\n存了 {len(rows)} 帧 + index.csv 于 {session}")
    print("【找焦】固定一档亮的快门/增益、扫 --lens-positions、对准已知距离标靶(给 --distance),")
    print("       取 lap_var 峰值的 lens 值=该距离对焦;多测几个距离 = 距离→对焦 标定表。")
    print("【调快门】锁定对焦后扫 --shutters:①肉眼看运动条纹越短越好;")
    print("        ②lap_var 只在亮度相近时比;③亮度太低=进光不够,升增益/加光/放长快门 权衡。")


if __name__ == "__main__":
    main()
