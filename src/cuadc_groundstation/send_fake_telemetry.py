#!/usr/bin/env python3
"""Send fake DroneLink telemetry UDP packets for ground-station UI testing.

Usage (on the Windows ground PC, with the localhost viewer already running):

    python windows\\send_fake_telemetry.py
    python windows\\send_fake_telemetry.py --host 127.0.0.1 --port 5701
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
            payload = {
                "t": time.time(),
                "alt_m": 10.0 + 2.0 * math.sin(t / 3.0),
                "roll_deg": 5.0 * math.sin(t),
                "pitch_deg": 3.0 * math.cos(t * 0.7),
                "yaw_deg": (t * 20.0) % 360.0,
                "mode": "GUIDED",
                "batt_v": 22.0 - 0.01 * t,
                "batt_pct": max(0.0, 90.0 - 0.05 * t),
            }
            sock.sendto(json.dumps(payload).encode("utf-8"), (args.host, args.port))
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
