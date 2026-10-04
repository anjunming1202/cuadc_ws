#!/usr/bin/env python3
"""台架：只监听飞控是否解锁。解锁/上锁必须用遥控器。不发 arm、不起飞。

用法（桨已拆，扇出在跑，不要占 14550 的 interface_bringup）：
  /home/wly/dronelink-venv/bin/python3 ~/flight_ctrl_tests/bench_motors.py
"""
from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil


def _armed(hb) -> bool:
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def main() -> int:
    ap = argparse.ArgumentParser(description="Watch RC arm/disarm; never sends arm")
    ap.add_argument("--device", default="udpin:127.0.0.1:14550")
    ap.add_argument("--timeout", type=float, default=180.0, help="seconds to wait for arm then disarm")
    args = ap.parse_args()

    print(f"connecting {args.device} (listen only, no arm command)", flush=True)
    m = mavutil.mavlink_connection(args.device, source_system=251, source_component=191)
    hb = m.wait_heartbeat(timeout=15)
    if hb is None:
        print("no heartbeat — is fanout up? is interface_bringup occupying 14550?", file=sys.stderr)
        return 1
    print(
        f"heartbeat sys={m.target_system} armed={_armed(hb)} "
        f"— use RC to ARM (props off), then DISARM",
        flush=True,
    )

    deadline = time.monotonic() + args.timeout
    saw_armed = False
    while time.monotonic() < deadline:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg is None:
            continue
        if _armed(msg):
            if not saw_armed:
                print("ARMED — motors should spin. Disarm with RC when done.", flush=True)
                saw_armed = True
        elif saw_armed:
            print("DISARMED — motor bench step ok", flush=True)
            return 0

    print("timeout waiting for RC arm then disarm", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
