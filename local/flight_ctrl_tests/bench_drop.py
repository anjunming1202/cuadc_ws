#!/usr/bin/env python3
"""台架投放：MAV_CMD_DO_SET_SERVO → SERVO9(=AUX1)。不解锁、不起飞。

硬件验证链路是 DO_SET_SERVO，不是 RC 通道 9 override（RC9 ≠ SERVO9；
MAVLink1 的 override 也发不到第 9 通道）。飞控需 SERVO9_FUNCTION=0。
AUX 舵机要单独供电、与飞控共地。

  /home/wly/dronelink-venv/bin/python3 ~/flight_ctrl_tests/bench_drop.py
"""
from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil


def _set_servo(m, servo: int, pwm: int) -> tuple:
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, servo, pwm, 0, 0, 0, 0, 0)
    deadline = time.monotonic() + 0.8
    ack = None
    out = None
    while time.monotonic() < deadline:
        msg = m.recv_match(
            type=["COMMAND_ACK", "SERVO_OUTPUT_RAW"], blocking=True, timeout=0.1)
        if msg is None:
            continue
        if msg.get_type() == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
            ack = msg.result
        elif msg.get_type() == "SERVO_OUTPUT_RAW":
            out = getattr(msg, f"servo{servo}_raw", None)
    return ack, out


def main() -> int:
    ap = argparse.ArgumentParser(description="Bench drop via DO_SET_SERVO on SERVO9; no arm")
    ap.add_argument("--device", default="udpin:127.0.0.1:14550")
    ap.add_argument("--servo", type=int, default=9, help="ArduPilot SERVO n (AUX1=9 on Pixhawk 6C)")
    ap.add_argument("--forward-us", type=int, default=1100)
    ap.add_argument("--neutral-us", type=int, default=1500)
    ap.add_argument("--run-s", type=float, default=1.975)
    args = ap.parse_args()

    print(f"connecting {args.device} (DO_SET_SERVO {args.servo}, no arm)", flush=True)
    m = mavutil.mavlink_connection(args.device, source_system=251, source_component=191)
    hb = m.wait_heartbeat(timeout=15)
    if hb is None:
        print("no heartbeat — fanout down or 14550 busy", file=sys.stderr)
        return 1
    print(f"heartbeat sys={m.target_system} comp={m.target_component}", flush=True)

    ack0, out0 = _set_servo(m, args.servo, args.neutral_us)
    print(f"neutral {args.neutral_us}us ack={ack0} servo_out={out0}", flush=True)

    t_end = time.monotonic() + args.run_s
    ack_f = out_f = None
    while time.monotonic() < t_end:
        ack_f, out_f = _set_servo(m, args.servo, args.forward_us)
        time.sleep(0.15)
    print(f"forward {args.forward_us}us x {args.run_s}s last_ack={ack_f} servo_out={out_f}", flush=True)

    ack1, out1 = _set_servo(m, args.servo, args.neutral_us)
    print(f"stop {args.neutral_us}us ack={ack1} servo_out={out1}", flush=True)
    print("done — ack 0=ACCEPTED; 2=DENIED (check SERVO9_FUNCTION=0, AUX power)", flush=True)
    if ack_f not in (0, None) and ack_f != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
