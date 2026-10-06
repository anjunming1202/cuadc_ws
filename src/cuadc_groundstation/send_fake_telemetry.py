#!/usr/bin/env python3
"""Send fake DroneLink telemetry UDP packets for ground-station UI testing.

Usage (on the Windows ground PC, with the localhost viewer already running):

    python send_fake_telemetry.py
    python send_fake_telemetry.py --host 127.0.0.1 --port 5701
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time


def main() -> None:
    parser = argparse.ArgumentParser(description="Fake DroneLink telemetry sender")
    parser.add_argument("--host", default="127.0.0.1", help="ground station IP")
    parser.add_argument("--port", type=int, default=5701, help="telemetry UDP port")
    parser.add_argument("--hz", type=float, default=10.0, help="send rate")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / max(args.hz, 0.1)
    t0 = time.time()
    print(f"Sending fake telemetry to {args.host}:{args.port} at {args.hz} Hz. Ctrl+C to stop.")
    try:
        while True:
            t = time.time() - t0
            armed = t >= 6.0  # 前 6s 未解锁：NED 乱走也不该离 H，高度钉 0
            # alt_m 模拟开机原点高度（解锁时约 -2.1）；地面站应显示相对解锁点
            payload = {
                "t": time.time(),
                "alt_m": -2.1 if not armed else -2.1 + 3.0 + 0.4 * math.sin((t - 6.0) / 3.0),
                "roll_deg": 5.0 * math.sin(t),
                "pitch_deg": 3.0 * math.cos(t * 0.7),
                "yaw_deg": 0.0,  # 北；与起飞机头一致时赛道 +x = 北
                "mode": "GUIDED" if armed else "STABILIZE",
                "armed": armed,
                "batt_v": 22.0 - 0.01 * t,
                "batt_pct": max(0.0, 90.0 - 0.05 * t),
                "ekf_ready": True,
                "gps_fix_type": 3,
                "gps_sats": 12,
                "ned_n": 3.0 * math.sin(t) if not armed else (t - 6.0) * 2.0,  # 解锁后沿北走
                "ned_e": 2.0 * math.cos(t) if not armed else 0.0,
                "cpu_pct": 55.0 + 20.0 * math.sin(t / 2.0),
            }
            sock.sendto(json.dumps(payload).encode("utf-8"), (args.host, args.port))
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
