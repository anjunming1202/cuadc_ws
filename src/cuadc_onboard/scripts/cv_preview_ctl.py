#!/usr/bin/env python3
"""cuadc_onboard CV 预览开关 —— 地面 viewer 按钮经 UDP 启停检测进程。

默认不跑 OpenCV(省 CPU)。收到 JSON `{cmd:on|off|status}` 后启停 worker
(`python3 -m cuadc_perception.cv_preview`,经 PYTHONPATH 引用,见 ADR-0008)
和 5604→5606 转发,并把 `{ok,running}` 回给来源地址。控制口对外(Wi-Fi)。
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time

DEFAULT_PORT = 5611  # 地面 viewer UDP 控制口;勿与 5610 相机控制冲突
WORKER_MODULE = "cuadc_perception.cv_preview"  # 检测代码在 cuadc_perception,按 PYTHONPATH 引用
FORWARD_CMD = [
    "gst-launch-1.0",
    "udpsrc", "port=5604",
    "!", "udpsink", "host=192.168.50.2", "port=5606",
    "sync=false", "async=false",
]


def _running(proc: subprocess.Popen | None) -> bool:
    return proc is not None and proc.poll() is None


class PreviewCtl:
    def __init__(self, env: dict[str, str], worker_args: list[str]) -> None:
        self._env = env
        self._args = worker_args
        self._proc: subprocess.Popen | None = None
        self._fwd: subprocess.Popen | None = None
        self._log = open("/tmp/cv_preview.log", "ab", buffering=0)  # 现场同一日志路径

    def _stop_one(self, proc: subprocess.Popen | None, label: str) -> None:
        if not _running(proc):
            return
        assert proc is not None
        pid = proc.pid
        try:
            os.killpg(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=2)
        print(f"[cv-ctl] stopped {label} pid={pid}", flush=True)

    def status(self) -> dict:
        live = _running(self._proc)
        if not live:
            self._proc = None
        if not _running(self._fwd):
            self._fwd = None
        return {
            "ok": True,
            "running": live,
            "pid": None if self._proc is None else self._proc.pid,
            "forward": _running(self._fwd),
        }

    def start(self) -> dict:
        if _running(self._proc):
            return self.status()
        cmd = ["python3", "-m", WORKER_MODULE, *self._args]
        self._proc = subprocess.Popen(
            cmd,
            env=self._env,
            stdout=self._log,
            stderr=subprocess.STDOUT,
            start_new_session=True,  # 独立进程组,stop 时整组杀掉
        )
        time.sleep(0.3)
        if not _running(self._proc):
            code = self._proc.returncode
            self._proc = None
            return {"ok": False, "running": False, "error": f"worker exited {code}"}
        self._fwd = subprocess.Popen(
            FORWARD_CMD,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        print(f"[cv-ctl] started worker={self._proc.pid} forward={self._fwd.pid}", flush=True)
        return self.status()

    def stop(self) -> dict:
        self._stop_one(self._fwd, "forward")
        self._fwd = None
        self._stop_one(self._proc, "worker")
        self._proc = None
        return self.status()


def main() -> None:
    ap = argparse.ArgumentParser(description="UDP 开关 OpenCV 预览(默认关)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = ap.parse_args()

    env = os.environ.copy()
    env.setdefault("GST_PLUGIN_PATH", "/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0")
    env.setdefault("PYTHONPATH", "/home/wly/cuadc_ws/src/cuadc_perception")
    worker_args = [
        "--from-rtp-port", "5602",
        "--src-fps", "10",
        "--port", "5604",
        "--width", "960",
        "--height", "540",
        "--bitrate", "1000",
        "--profile", "real",
    ]
    ctl = PreviewCtl(env, worker_args)

    def _shutdown(_signum, _frame) -> None:
        ctl.stop()
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))
    print(f"[cv-ctl] udp {args.host}:{args.port} worker=-m {WORKER_MODULE} (default off)", flush=True)

    while True:
        try:
            data, addr = sock.recvfrom(4096)
            msg = json.loads(data.decode("utf-8"))
            cmd = str(msg.get("cmd") or "").lower()
            if cmd in ("on", "start") or msg.get("enable") is True:
                reply = ctl.start()
            elif cmd in ("off", "stop") or msg.get("enable") is False:
                reply = ctl.stop()
            else:
                reply = ctl.status()
            sock.sendto(json.dumps(reply).encode("utf-8"), addr)
        except Exception as exc:  # 单条坏命令不退出
            print(f"[cv-ctl] bad command: {exc}", file=sys.stderr, flush=True)
            try:
                sock.sendto(json.dumps({"ok": False, "running": False, "error": str(exc)}).encode(), addr)
            except Exception:
                pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
