#!/usr/bin/env python3
"""cuadc_onboard 干净图传开关 —— 地面 viewer 按钮经 UDP 启停 5603→5600。

与 cv_preview_ctl 同协议：JSON `{cmd:on|off|status}` → `{ok,running}`。
控制口 5612。进程启动时默认 **on**（开机自启即有原始图传）。
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

DEFAULT_PORT = 5612  # 地面图传控制口；勿与 5610 相机 / 5611 CV 冲突
FORWARD_CMD = [
    "gst-launch-1.0",
    "udpsrc", "port=5603",
    "!", "udpsink", "host=192.168.50.2", "port=5600",
    "sync=false", "async=false",
]


def _running(proc: subprocess.Popen | None) -> bool:
    return proc is not None and proc.poll() is None


class VideoForwardCtl:
    def __init__(self) -> None:
        self._fwd: subprocess.Popen | None = None

    def _stop_one(self, proc: subprocess.Popen | None) -> None:
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
        print(f"[vf-ctl] stopped forward pid={pid}", flush=True)

    def status(self) -> dict:
        live = _running(self._fwd)
        if not live:
            self._fwd = None
        return {
            "ok": True,
            "running": live,
            "pid": None if self._fwd is None else self._fwd.pid,
        }

    def start(self) -> dict:
        if _running(self._fwd):
            return self.status()
        self._fwd = subprocess.Popen(
            FORWARD_CMD,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        time.sleep(0.2)
        if not _running(self._fwd):
            code = self._fwd.returncode
            self._fwd = None
            return {"ok": False, "running": False, "error": f"gst exited {code}"}
        print(f"[vf-ctl] started forward pid={self._fwd.pid}", flush=True)
        return self.status()

    def stop(self) -> dict:
        self._stop_one(self._fwd)
        self._fwd = None
        return self.status()


def main() -> None:
    ap = argparse.ArgumentParser(description="UDP 开关干净图传(默认开)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument(
        "--default-on",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="进程起来后是否立刻开转发(默认开)",
    )
    args = ap.parse_args()

    ctl = VideoForwardCtl()

    def _shutdown(_signum, _frame) -> None:
        ctl.stop()
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    if args.default_on:
        ctl.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))
    print(
        f"[vf-ctl] udp {args.host}:{args.port} "
        f"(default {'on' if args.default_on else 'off'})",
        flush=True,
    )

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
            print(f"[vf-ctl] bad command: {exc}", file=sys.stderr, flush=True)
            try:
                sock.sendto(
                    json.dumps({"ok": False, "running": False, "error": str(exc)}).encode(),
                    addr,
                )
            except Exception:
                pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
