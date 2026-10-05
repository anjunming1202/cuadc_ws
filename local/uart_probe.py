#!/usr/bin/env python3
"""Brief serial probe: AMA0/AMA10 x 57600/115200. Not a flight command."""
import sys
from pymavlink import mavutil

def try_port(dev: str, baud: int) -> None:
    print(f"TRY {dev} @{baud}", flush=True)
    try:
        m = mavutil.mavlink_connection(
            dev, baud=baud, source_system=250, source_component=190
        )
    except Exception as exc:
        print(f"  open failed: {exc}", flush=True)
        return
    hb = m.wait_heartbeat(timeout=8)
    if hb is None:
        print("  no heartbeat", flush=True)
    else:
        print(f"  HEARTBEAT sys={m.target_system} comp={m.target_component} type={getattr(hb, 'type', None)}", flush=True)
    try:
        m.close()
    except Exception:
        pass


def main() -> None:
    for dev in ("/dev/ttyAMA0", "/dev/ttyAMA10"):
        for baud in (57600, 115200):
            try_port(dev, baud)


if __name__ == "__main__":
    sys.exit(main())
