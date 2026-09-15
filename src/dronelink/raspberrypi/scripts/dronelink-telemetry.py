#!/usr/bin/env python3
"""DroneLink air-side telemetry: Pixhawk MAVLink -> UDP JSON to ground.

No ROS. Reads /dev/ttyAMA0 (TELEM->GPIO UART) and sends one JSON datagram
per tick to the Windows ground station (default 192.168.50.2:5701), matching
the schema expected by windows/localhost_viewer.

Migration note (later, when cuadc_interface runs on the Pi):
  - Stop this service first — only one process may own the serial port.
  - Replace collect_from_mavlink() with subscriptions to
    /cuadc_interface/telemetry/* and keep encode_payload() + UDP send
    unchanged so the ground HUD does not need a new protocol.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from typing import Any

from pymavlink import mavutil

DEFAULT_DEVICE = "/dev/ttyAMA0"
DEFAULT_BAUD = 57600
DEFAULT_DEST_HOST = "192.168.50.2"
DEFAULT_DEST_PORT = 5701  # avoid FFmpeg RTCP on :5601 (video RTP uses :5600)
DEFAULT_HZ = 10.0


def encode_payload(state: dict[str, Any]) -> dict[str, Any]:
    """Map internal state -> ground HUD JSON. Keep stable across ROS migration."""
    return {
        "t": time.time(),
        "alt_m": state.get("alt_m"),
        "roll_deg": state.get("roll_deg"),
        "pitch_deg": state.get("pitch_deg"),
        "yaw_deg": state.get("yaw_deg"),
        "mode": state.get("mode"),
        "batt_v": state.get("batt_v"),
        "batt_pct": state.get("batt_pct"),
    }


def _unwrap_deg(rad: float) -> float:
    return math.degrees(rad)


class MavlinkTelemetrySource:
    """Owns the serial MAVLink link. Swap this class out when moving to ROS."""

    def __init__(self, device: str, baud: int) -> None:
        self._device = device
        self._baud = baud
        self._master: Any = None
        self._state: dict[str, Any] = {
            "alt_m": None,
            "roll_deg": None,
            "pitch_deg": None,
            "yaw_deg": None,
            "mode": None,
            "batt_v": None,
            "batt_pct": None,
        }

    def connect(self) -> None:
        conn = f"{self._device}:{self._baud}"
        print(f"Connecting to {conn} ...", flush=True)
        self._master = mavutil.mavlink_connection(self._device, baud=self._baud)
        self._master.wait_heartbeat(timeout=30)
        print(
            f"Heartbeat from system {self._master.target_system} "
            f"component {self._master.target_component}",
            flush=True,
        )
        # Ask ArduPilot for a steady stream; harmless if already streaming.
        self._master.mav.request_data_stream_send(
            self._master.target_system,
            self._master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1,
        )

    def pump(self, budget_s: float = 0.05) -> None:
        """Drain available MAVLink messages for up to budget_s seconds."""
        if self._master is None:
            return
        deadline = time.monotonic() + budget_s
        while time.monotonic() < deadline:
            msg = self._master.recv_match(blocking=False)
            if msg is None:
                break
            self._handle(msg)

    def snapshot(self) -> dict[str, Any]:
        return dict(self._state)

    def _handle(self, msg: Any) -> None:
        mtype = msg.get_type()
        if mtype == "ATTITUDE":
            self._state["roll_deg"] = _unwrap_deg(msg.roll)
            self._state["pitch_deg"] = _unwrap_deg(msg.pitch)
            self._state["yaw_deg"] = _unwrap_deg(msg.yaw) % 360.0
        elif mtype == "LOCAL_POSITION_NED":
            # Same convention as cuadc_interface: z down positive -> alt = -z
            self._state["alt_m"] = float(-msg.z)
        elif mtype == "GLOBAL_POSITION_INT":
            # Fallback if LOCAL_POSITION_NED is slow/absent (relative_alt in mm)
            if self._state["alt_m"] is None and msg.relative_alt is not None:
                self._state["alt_m"] = msg.relative_alt / 1000.0
        elif mtype == "HEARTBEAT":
            # Skip GCS / companion heartbeats that may appear on the bus
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                return
            try:
                self._state["mode"] = mavutil.mode_string_v10(msg)
            except Exception:
                self._state["mode"] = str(msg.custom_mode)
        elif mtype == "SYS_STATUS":
            if msg.voltage_battery > 0:
                self._state["batt_v"] = msg.voltage_battery / 1000.0
            if 0 <= msg.battery_remaining <= 100:
                self._state["batt_pct"] = float(msg.battery_remaining)
        elif mtype == "BATTERY_STATUS":
            if msg.battery_remaining >= 0:
                self._state["batt_pct"] = float(msg.battery_remaining)
            voltages = getattr(msg, "voltages", None)
            if voltages and voltages[0] > 0 and voltages[0] < 65535:
                self._state["batt_v"] = voltages[0] / 1000.0


def main() -> None:
    parser = argparse.ArgumentParser(description="DroneLink MAVLink -> UDP telemetry")
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--dest-host", default=DEFAULT_DEST_HOST)
    parser.add_argument("--dest-port", type=int, default=DEFAULT_DEST_PORT)
    parser.add_argument("--hz", type=float, default=DEFAULT_HZ)
    args = parser.parse_args()

    source = MavlinkTelemetrySource(args.device, args.baud)
    try:
        source.connect()
    except Exception as exc:
        print(f"Failed to connect: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1) from exc

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.dest_host, args.dest_port)
    period = 1.0 / max(args.hz, 0.1)
    print(f"Sending telemetry to {dest[0]}:{dest[1]} at {args.hz} Hz", flush=True)

    try:
        while True:
            t0 = time.monotonic()
            source.pump(budget_s=min(0.05, period * 0.8))
            payload = encode_payload(source.snapshot())
            # Drop keys that are still None so the HUD shows "—" cleanly
            compact = {k: v for k, v in payload.items() if v is not None or k == "t"}
            sock.sendto(json.dumps(compact).encode("utf-8"), dest)
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period - elapsed))
    except KeyboardInterrupt:
        print("Stopped.", flush=True)


if __name__ == "__main__":
    main()
