#!/usr/bin/env python3
"""DroneLink air-side telemetry: Pixhawk MAVLink -> UDP JSON to ground.

No ROS. Reads /dev/ttyAMA0 (TELEM->GPIO UART) and sends one JSON datagram
per tick to the Windows ground station (default 192.168.50.2:5701 and
10.10.10.2:5701). Ground viewer also accepts legacy :5601.

Boot robustness: never exit just because the FC is late after power-up;
reconnect and re-request streams until data flows. systemd Restart=always
is the backstop.
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
DEFAULT_DEST_HOST = "192.168.50.2,10.10.10.2" # Wi-Fi + Ethernet; comma-separated
DEFAULT_DEST_PORT = 5701
DEFAULT_HZ = 10.0

_MSG_INTERVALS_US = {
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 100000,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 100000,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 200000,
    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 1000000,
    mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS: 1000000,
    mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD: 200000,
}


def encode_payload(state: dict[str, Any]) -> dict[str, Any]:
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
    def __init__(self, device: str, baud: int) -> None:
        self._device = device
        self._baud = baud
        self._master: Any = None
        self._last_stream_req = 0.0
        self._state: dict[str, Any] = self._empty_state()
        self._seen: set[str] = set()

    @staticmethod
    def _empty_state() -> dict[str, Any]:
        return {
            "alt_m": None,
            "roll_deg": None,
            "pitch_deg": None,
            "yaw_deg": None,
            "mode": None,
            "batt_v": None,
            "batt_pct": None,
        }

    def close(self) -> None:
        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None

    def connect(self, heartbeat_timeout: float = 30.0) -> None:
        self.close()
        self._state = self._empty_state()
        self._seen = set()
        print(f"Connecting to {self._device}:{self._baud} ...", flush=True)
        self._master = mavutil.mavlink_connection(self._device, baud=self._baud)
        hb = self._master.wait_heartbeat(timeout=heartbeat_timeout)
        if hb is None:
            self.close()
            raise TimeoutError("no MAVLink heartbeat")
        self._apply_heartbeat(hb, force_targets=True)
        self._prefer_autopilot_targets(timeout=8.0)
        if not self._master.target_system:
            self._master.target_system = 1
        if not self._master.target_component:
            self._master.target_component = 1
        print(
            f"Using MAVLink target {self._master.target_system}/"
            f"{self._master.target_component} mode={self._state.get('mode')}",
            flush=True,
        )
        self._request_streams()

    def _prefer_autopilot_targets(self, timeout: float) -> None:
        assert self._master is not None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg is None:
                continue
            self._apply_heartbeat(msg, force_targets=True)
            if msg.get_srcSystem() and msg.type not in (
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_TYPE_GENERIC,
            ):
                return
            if msg.get_srcSystem():
                return

    def _apply_heartbeat(self, msg: Any, force_targets: bool = False) -> None:
        if msg is None:
            return
        assert self._master is not None
        src_sys = msg.get_srcSystem()
        src_comp = msg.get_srcComponent()
        if force_targets and src_sys:
            self._master.target_system = src_sys
            self._master.target_component = src_comp or 1
        if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
            return
        try:
            self._state["mode"] = mavutil.mode_string_v10(msg)
        except Exception:
            self._state["mode"] = str(getattr(msg, "custom_mode", "?"))

    def _request_streams(self) -> None:
        assert self._master is not None
        sysid = self._master.target_system or 1
        compid = self._master.target_component or 1
        self._master.mav.request_data_stream_send(
            sysid, compid, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
        )
        for stream_id in (
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
        ):
            self._master.mav.request_data_stream_send(sysid, compid, stream_id, 10, 1)
        for msg_id, interval_us in _MSG_INTERVALS_US.items():
            self._master.mav.command_long_send(
                sysid,
                compid,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(msg_id),
                float(interval_us),
                0,
                0,
                0,
                0,
                0,
            )
        self._last_stream_req = time.monotonic()
        print(f"Requested data streams from {sysid}/{compid}", flush=True)

    def has_motion_fields(self) -> bool:
        return self._state.get("roll_deg") is not None or self._state.get("alt_m") is not None

    def pump(self, budget_s: float = 0.08) -> None:
        if self._master is None:
            return
        now = time.monotonic()
        # Re-request often until attitude/alt appear; then every 5s.
        interval = 2.0 if not self.has_motion_fields() else 5.0
        if now - self._last_stream_req > interval:
            self._request_streams()
        deadline = time.monotonic() + budget_s
        while time.monotonic() < deadline:
            msg = self._master.recv_match(blocking=False)
            if msg is None:
                time.sleep(0.001)
                continue
            self._handle(msg)

    def snapshot(self) -> dict[str, Any]:
        return dict(self._state)

    def _handle(self, msg: Any) -> None:
        mtype = msg.get_type()
        if mtype in ("BAD_DATA", "UNKNOWN"):
            return
        if mtype not in self._seen:
            self._seen.add(mtype)
            if mtype != "HEARTBEAT":
                print(f"First {mtype} received", flush=True)

        if mtype == "ATTITUDE":
            self._state["roll_deg"] = _unwrap_deg(msg.roll)
            self._state["pitch_deg"] = _unwrap_deg(msg.pitch)
            self._state["yaw_deg"] = _unwrap_deg(msg.yaw) % 360.0
        elif mtype == "LOCAL_POSITION_NED":
            self._state["alt_m"] = float(-msg.z)
        elif mtype == "GLOBAL_POSITION_INT":
            self._state["alt_m"] = msg.relative_alt / 1000.0
        elif mtype == "VFR_HUD":
            if self._state["alt_m"] is None:
                self._state["alt_m"] = float(msg.alt)
        elif mtype == "HEARTBEAT":
            self._apply_heartbeat(msg, force_targets=False)
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


def run_forever(args: argparse.Namespace) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Comma-separated hosts: send the same JSON to each (Wi-Fi + Ethernet backup).
    hosts = [h.strip() for h in args.dest_host.split(",") if h.strip()]
    dests = [(h, args.dest_port) for h in hosts]
    period = 1.0 / max(args.hz, 0.1)
    source = MavlinkTelemetrySource(args.device, args.baud)
    backoff = 2.0

    while True:
        try:
            source.connect(heartbeat_timeout=45.0)
            backoff = 2.0
            print(
                f"Sending telemetry to {', '.join(f'{h}:{args.dest_port}' for h,_ in dests)} "
                f"at {args.hz} Hz",
                flush=True,
            )
            last_data_log = 0.0
            while True:
                t0 = time.monotonic()
                source.pump(budget_s=min(0.08, period * 0.9))
                payload = encode_payload(source.snapshot())
                compact = {k: v for k, v in payload.items() if v is not None or k == "t"}
                blob = json.dumps(compact).encode("utf-8")
                for dest in dests:
                    try:
                        sock.sendto(blob, dest)
                    except OSError as exc:
                        print(f"sendto {dest} failed: {exc}", file=sys.stderr, flush=True)
                if time.monotonic() - last_data_log > 10.0:
                    last_data_log = time.monotonic()
                    print(
                        "health: mode={mode} alt={alt} roll={roll} keys={keys}".format(
                            mode=compact.get("mode"),
                            alt=compact.get("alt_m"),
                            roll=compact.get("roll_deg"),
                            keys=sorted(k for k in compact if k != "t"),
                        ),
                        flush=True,
                    )
                elapsed = time.monotonic() - t0
                time.sleep(max(0.0, period - elapsed))
        except KeyboardInterrupt:
            print("Stopped.", flush=True)
            source.close()
            return
        except Exception as exc:
            print(f"Link error: {exc}; retry in {backoff:.0f}s", file=sys.stderr, flush=True)
            source.close()
            time.sleep(backoff)
            backoff = min(30.0, backoff * 1.5)


def main() -> None:
    parser = argparse.ArgumentParser(description="DroneLink MAVLink -> UDP telemetry")
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--dest-host",
        default=DEFAULT_DEST_HOST,
        help="comma-separated ground IPs (DroneLink Wi-Fi and/or Ethernet)",
    )
    parser.add_argument("--dest-port", type=int, default=DEFAULT_DEST_PORT)
    parser.add_argument("--hz", type=float, default=DEFAULT_HZ)
    args = parser.parse_args()
    run_forever(args)


if __name__ == "__main__":
    main()
