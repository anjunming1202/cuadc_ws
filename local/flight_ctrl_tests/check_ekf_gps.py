#!/usr/bin/env python3
"""Print EKF/GPS ready from fanout. No arm. Uses udpin:127.0.0.1:14550."""
from __future__ import annotations

import sys
import time

from pymavlink import mavutil

_READY = (
    mavutil.mavlink.EKF_POS_HORIZ_ABS
    | mavutil.mavlink.EKF_POS_VERT_ABS
)

dev = sys.argv[1] if len(sys.argv) > 1 else "udpin:127.0.0.1:14550"
print(f"connecting {dev}", flush=True)
m = mavutil.mavlink_connection(dev, source_system=252, source_component=192)
if m.wait_heartbeat(timeout=10) is None:
    print("no heartbeat — fanout down or 14550 busy", file=sys.stderr)
    sys.exit(1)
m.mav.request_data_stream_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

ekf = gps = None
deadline = time.monotonic() + 12.0
while time.monotonic() < deadline:
    msg = m.recv_match(
        type=["EKF_STATUS_REPORT", "GPS_RAW_INT", "GPS2_RAW"],
        blocking=True, timeout=1.0)
    if msg is None:
        continue
    t = msg.get_type()
    if t == "EKF_STATUS_REPORT":
        ekf = msg
    elif t in ("GPS_RAW_INT", "GPS2_RAW"):
        gps = msg
    if ekf is not None and gps is not None:
        break

if ekf is None:
    print("no EKF_STATUS_REPORT yet")
else:
    ready = (ekf.flags & _READY) == _READY
    print(
        f"EKF flags=0x{ekf.flags:04x} "
        f"horiz_abs={bool(ekf.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS)} "
        f"vert_abs={bool(ekf.flags & mavutil.mavlink.EKF_POS_VERT_ABS)} "
        f"-> ready={ready}"
    )
if gps is None:
    print("no GPS_RAW_INT yet")
else:
    # fix_type: 0=no, 2=2D, 3=3D, 4=DGPS, 5=RTK float, 6=RTK fixed
    print(
        f"GPS fix_type={gps.fix_type} sats={gps.satellites_visible} "
        f"eph={getattr(gps, 'eph', None)} epv={getattr(gps, 'epv', None)}"
    )

if ekf is not None and (ekf.flags & _READY) == _READY:
    print("VERDICT: EKF/GPS READY")
    sys.exit(0)
print("VERDICT: NOT READY")
sys.exit(1)
