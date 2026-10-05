#!/usr/bin/env python3
"""One-shot heartbeat probe. Does not arm. Usage: probe_hb_once.py /dev/ttyAMA0 57600"""
import sys
import time
from pymavlink import mavutil

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA0"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 57600
print(f"probe {dev} @{baud}", flush=True)
m = mavutil.mavlink_connection(dev, baud=baud, source_system=250, source_component=190)
t0 = time.time()
hb = m.wait_heartbeat(timeout=8)
dt = time.time() - t0
if hb is None:
    print(f"no heartbeat after {dt:.1f}s", flush=True)
    code = 1
else:
    print(
        f"HEARTBEAT sys={m.target_system} comp={m.target_component} "
        f"type={getattr(hb, 'type', None)} {dt:.1f}s",
        flush=True,
    )
    code = 0
try:
    m.close()
except Exception:
    pass
sys.exit(code)
