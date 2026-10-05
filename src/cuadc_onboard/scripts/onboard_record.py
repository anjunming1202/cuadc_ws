#!/usr/bin/env python3
"""cuadc_onboard 机载录制 —— 全质量视频(相机 tee 5605)+ rosbag,共享后端 Recorder。

**不自启**(开发/写代码时不会录空 bag)。前端:
- 本脚本 CLI:**前台跑、Ctrl-C 停**,ssh 进 Pi 一条命令即录。
- 以后的网络触发(地面站按钮 → Pi 控制口)`import Recorder` 复用**同一后端**,不另造逻辑。

和任务/测试代码解耦:视频靠相机服务的 tee 5605 分支、rosbag 靠 ROS 话题,谁在跑都能录。
产物落 <dir>/<时间戳>[_label].ts + <时间戳>[_label]_bag/,落地后 pull 回 local/recordings/。

    python3 onboard_record.py                 # 视频 + rosbag(全话题),Ctrl-C 停
    python3 onboard_record.py --no-bag        # 只视频
    python3 onboard_record.py --topics /a /b  # rosbag 只录指定话题
"""
from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import signal
import subprocess
import sys
import time
from pathlib import Path

VIDEO_PORT = 5605
VIDEO_CAPS = "application/x-rtp,media=video,encoding-name=H264,payload=96"


def _set_pdeathsig() -> None:
    """子进程 preexec:父进程一死,内核给本子进程发 SIGTERM —— 防 ssh 断线/父被杀时
    gst/ros2 变孤儿继续占住 5604(下次录制就废)。"""
    try:
        libc = ctypes.CDLL(ctypes.util.find_library("c") or "libc.so.6", use_errno=True)
        libc.prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG = 1
    except Exception:
        pass


class Recorder:
    """机载录制后端:start() 拉起视频(gst)+rosbag(ros2 bag)子进程,stop() 发 SIGINT 干净收尾。
    子进程设 PR_SET_PDEATHSIG:父进程一死就自杀,绝不留孤儿占 5604/相机。"""

    def __init__(self, out_dir: Path, video: bool = True, bag: bool = True,
                 topics: list[str] | None = None) -> None:
        self._dir = Path(out_dir)
        self._video = video
        self._bag = bag
        self._topics = topics or []
        self._procs: list[tuple[str, subprocess.Popen]] = []
        self.prefix: str | None = None

    def start(self, label: str | None = None) -> str:
        self._dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.prefix = f"{stamp}_{label}" if label else stamp
        if self._video:
            ts = self._dir / f"{self.prefix}.ts"
            cmd = ["gst-launch-1.0", "-e", "-q",
                   "udpsrc", f"port={VIDEO_PORT}", f"caps={VIDEO_CAPS}",
                   "!", "rtpjitterbuffer", "latency=200",
                   "!", "rtph264depay", "!", "h264parse",
                   "!", "mpegtsmux", "!", "filesink", f"location={ts}"]
            self._procs.append(("video", subprocess.Popen(cmd, preexec_fn=_set_pdeathsig)))
        if self._bag:
            bagdir = self._dir / f"{self.prefix}_bag"
            cmd = ["ros2", "bag", "record", "-o", str(bagdir)]
            cmd += self._topics if self._topics else ["-a"]
            self._procs.append(("bag", subprocess.Popen(cmd, preexec_fn=_set_pdeathsig)))
        return self.prefix

    def dead(self) -> list[str]:
        return [n for n, p in self._procs if p.poll() is not None]

    def stop(self) -> None:
        for _name, p in self._procs:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)  # gst -e -> EOS 收尾;ros2 bag -> 写 metadata.yaml
        for _name, p in self._procs:
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                p.kill()
        self._procs = []


def main() -> None:
    ap = argparse.ArgumentParser(description="机载录制(视频+rosbag);前台跑,Ctrl-C 停")
    ap.add_argument("--dir", type=Path, default=Path.home() / "flight_recordings")
    ap.add_argument("--label", default=None, help="文件名后缀标签")
    ap.add_argument("--no-video", action="store_true", help="不录视频(只 rosbag)")
    ap.add_argument("--no-bag", action="store_true", help="不录 rosbag(只视频)")
    ap.add_argument("--topics", nargs="*", default=None, help="rosbag 只录这些话题(默认 -a 全录)")
    args = ap.parse_args()

    rec = Recorder(args.dir, video=not args.no_video, bag=not args.no_bag, topics=args.topics)
    prefix = rec.start(args.label)

    def _graceful(_signum, _frame):
        raise KeyboardInterrupt   # SIGTERM / SIGHUP(ssh 断线) 也走优雅收尾

    signal.signal(signal.SIGTERM, _graceful)
    signal.signal(signal.SIGHUP, _graceful)
    print(f"[rec] 录制中 -> {args.dir}/{prefix}{{.ts,_bag}}   (Ctrl-C 停)", flush=True)
    try:
        while True:
            time.sleep(0.5)
            dead = rec.dead()
            if dead:
                print(f"[rec] 子进程提前退出: {dead};停止其余", file=sys.stderr)
                break
    except KeyboardInterrupt:
        pass
    print("\n[rec] 收尾中…", flush=True)
    rec.stop()
    print(f"[rec] 完成: {args.dir}/{prefix}")


if __name__ == "__main__":
    main()
