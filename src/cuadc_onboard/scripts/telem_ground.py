#!/usr/bin/env python3
"""DroneLink air-side telemetry: Pixhawk MAVLink -> UDP JSON to ground.

Sends immediately (even while waiting for the FC) so the ground HUD can
tell "no UDP" from "UDP ok, FC still coming up". Datagrams go to both
:5701 and legacy :5601, on Wi-Fi and Ethernet if configured.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time
from typing import Any

from pymavlink import mavutil

DEFAULT_DEVICE = "/dev/ttyAMA0"
DEFAULT_BAUD = 921600
DEFAULT_DEST_HOST = "192.168.50.2,10.10.10.2"  # Wi-Fi + Ethernet
DEFAULT_DEST_PORTS = "5701,5601"  # preferred + legacy (viewer demuxes 5601)
DEFAULT_HZ = 10.0

_MSG_INTERVALS_US = {
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 100000,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 100000,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 200000,
    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 1000000,
    mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS: 1000000,
    mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD: 200000,
    mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 500000,
    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 500000,
}

# 与 cuadc_interface / 验收脚本同一判据：水平+垂直绝对位置可信才算 ready。
_EKF_POS_READY = (
    mavutil.mavlink.EKF_POS_HORIZ_ABS | mavutil.mavlink.EKF_POS_VERT_ABS
)


def _unwrap_deg(rad: float) -> float:
    return math.degrees(rad)


class MavlinkTelemetrySource:
    def __init__(self, device: str, baud: int) -> None:
        self._device = device
        self._baud = baud
        self._master: Any = None
        self._last_stream_req = 0.0
        # 经 mav_fanout（udp/tcp 端点）时飞控数据流速率归扇出（唯一属主），这里只消费；
        # 只有自己直连串口时才请求数据流。
        self._owns_link = not device.startswith(("udp", "tcp"))
        self._lock = threading.Lock()
        self._state: dict[str, Any] = self._empty_state()
        self._seen: set[str] = set()
        self.link = "connecting"  # connecting | fc | error

    @staticmethod
    def _empty_state() -> dict[str, Any]:
        return {
            "alt_m": None,
            "roll_deg": None,
            "pitch_deg": None,
            "yaw_deg": None,
            "mode": None,
            "armed": None,  # 地面起飞坐标系在解锁上升沿锁原点
            "batt_v": None,
            "batt_pct": None,
            "ekf_ready": None,  # bool；未收到 EKF_STATUS_REPORT 前为 None
            "gps_fix_type": None,  # 0=无, 3=3D, …
            "gps_sats": None,
            "ned_n": None,  # LOCAL_POSITION_NED.x 北向 m；地面赛道图用
            "ned_e": None,  # LOCAL_POSITION_NED.y 东向 m
            "lat_deg": None,
            "lon_deg": None,
        }

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            out = dict(self._state)
            out["link"] = self.link
            return out

    def close(self) -> None:
        if self._master is not None:
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None

    def connect(self, heartbeat_timeout: float = 8.0) -> None:
        self.close()
        with self._lock:
            self._state = self._empty_state()
            self._seen = set()
            self.link = "connecting"
        print(f"Connecting to {self._device}:{self._baud} ...", flush=True)
        self._master = mavutil.mavlink_connection(self._device, baud=self._baud)
        hb = self._master.wait_heartbeat(timeout=heartbeat_timeout)
        if hb is None:
            self.close()
            raise TimeoutError("no MAVLink heartbeat")
        self._apply_heartbeat(hb, force_targets=True)
        if not self._master.target_system:
            self._master.target_system = 1
        if not self._master.target_component:
            self._master.target_component = 1
        with self._lock:
            self.link = "fc"
        print(
            f"Heartbeat sys={self._master.target_system}/"
            f"{self._master.target_component} mode={self.snapshot().get('mode')}",
            flush=True,
        )
        if self._owns_link:
            self._request_streams()

    def _apply_heartbeat(self, msg: Any, force_targets: bool = False) -> None:
        if msg is None:
            return
        assert self._master is not None
        src_sys = msg.get_srcSystem()
        src_comp = msg.get_srcComponent()
        if force_targets and src_sys:
            self._master.target_system = src_sys
            self._master.target_component = src_comp or 1
        if msg.type == mavutil.mavlink.MAV_TYPE_GCS or msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            return  # 只认飞控自己的心跳
        try:
            mode = mavutil.mode_string_v10(msg)
        except Exception:
            mode = str(getattr(msg, "custom_mode", "?"))
        with self._lock:
            self._state["mode"] = mode
            self._state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

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
        with self._lock:
            return self._state.get("roll_deg") is not None or self._state.get("alt_m") is not None

    def pump(self, budget_s: float = 0.08) -> None:
        if self._master is None:
            return
        now = time.monotonic()
        interval = 2.0 if not self.has_motion_fields() else 5.0
        if self._owns_link and now - self._last_stream_req > interval:
            self._request_streams()
        deadline = time.monotonic() + budget_s
        while time.monotonic() < deadline:
            msg = self._master.recv_match(blocking=False)
            if msg is None:
                time.sleep(0.001)
                continue
            self._handle(msg)

    def _handle(self, msg: Any) -> None:
        mtype = msg.get_type()
        if mtype in ("BAD_DATA", "UNKNOWN"):
            return
        if mtype not in self._seen:
            self._seen.add(mtype)
            if mtype != "HEARTBEAT":
                print(f"First {mtype} received", flush=True)

        if mtype == "ATTITUDE":
            with self._lock:
                self._state["roll_deg"] = _unwrap_deg(msg.roll)
                self._state["pitch_deg"] = _unwrap_deg(msg.pitch)
                self._state["yaw_deg"] = _unwrap_deg(msg.yaw) % 360.0
        elif mtype == "LOCAL_POSITION_NED":
            with self._lock:
                self._state["ned_n"] = float(msg.x)
                self._state["ned_e"] = float(msg.y)
                self._state["alt_m"] = float(-msg.z)
        elif mtype == "GLOBAL_POSITION_INT":
            with self._lock:
                self._state["alt_m"] = msg.relative_alt / 1000.0
                self._state["lat_deg"] = msg.lat / 1e7
                self._state["lon_deg"] = msg.lon / 1e7
        elif mtype == "VFR_HUD":
            with self._lock:
                if self._state["alt_m"] is None:
                    self._state["alt_m"] = float(msg.alt)
        elif mtype == "HEARTBEAT":
            self._apply_heartbeat(msg, force_targets=False)
        elif mtype == "SYS_STATUS":
            with self._lock:
                if msg.voltage_battery > 0:
                    self._state["batt_v"] = msg.voltage_battery / 1000.0
                if 0 <= msg.battery_remaining <= 100:
                    self._state["batt_pct"] = float(msg.battery_remaining)
        elif mtype == "BATTERY_STATUS":
            with self._lock:
                if msg.battery_remaining >= 0:
                    self._state["batt_pct"] = float(msg.battery_remaining)
                voltages = getattr(msg, "voltages", None)
                if voltages and voltages[0] > 0 and voltages[0] < 65535:
                    self._state["batt_v"] = voltages[0] / 1000.0
        elif mtype == "EKF_STATUS_REPORT":
            with self._lock:
                self._state["ekf_ready"] = (msg.flags & _EKF_POS_READY) == _EKF_POS_READY
        elif mtype in ("GPS_RAW_INT", "GPS2_RAW"):
            with self._lock:
                self._state["gps_fix_type"] = int(msg.fix_type)
                self._state["gps_sats"] = int(msg.satellites_visible)


def _dest_list(hosts: str, ports: str) -> list[tuple[str, int]]:
    hs = [h.strip() for h in hosts.split(",") if h.strip()]
    ps = [int(p.strip()) for p in ports.split(",") if p.strip()]
    dests: list[tuple[str, int]] = []
    for h in hs:
        for p in ps:
            dests.append((h, p))
    return dests


class HostCpuSampler:
    """Total host CPU % from /proc/stat deltas (all cores averaged)."""

    def __init__(self) -> None:
        self._idle = 0
        self._total = 0
        self._pct: float | None = None
        self._sample()

    @staticmethod
    def _read() -> tuple[int, int] | None:
        try:
            with open("/proc/stat", encoding="ascii") as f:
                parts = f.readline().split()
        except OSError:
            return None
        if not parts or parts[0] != "cpu" or len(parts) < 5:
            return None
        nums = [int(x) for x in parts[1:]]
        idle = nums[3] + (nums[4] if len(nums) > 4 else 0)  # idle + iowait
        return idle, sum(nums)

    def _sample(self) -> None:
        cur = self._read()
        if cur is None:
            return
        idle, total = cur
        d_total = total - self._total
        d_idle = idle - self._idle
        if self._total > 0 and d_total > 0:
            self._pct = max(0.0, min(100.0, 100.0 * (1.0 - d_idle / d_total)))
        self._idle, self._total = idle, total

    def percent(self) -> float | None:
        self._sample()
        return self._pct


def run_forever(args: argparse.Namespace) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dests = _dest_list(args.dest_host, args.dest_ports)
    period = 1.0 / max(args.hz, 0.1)
    source = MavlinkTelemetrySource(args.device, args.baud)
    cpu = HostCpuSampler()
    stop = threading.Event()
    print(
        "UDP to " + ", ".join(f"{h}:{p}" for h, p in dests) + f" at {args.hz} Hz",
        flush=True,
    )

    def send_loop() -> None:
        last_log = 0.0
        while not stop.is_set():
            t0 = time.monotonic()
            state = source.snapshot()
            cpu_pct = cpu.percent()
            payload = {
                "t": time.time(),
                "link": state.get("link"),
                "alt_m": state.get("alt_m"),
                "roll_deg": state.get("roll_deg"),
                "pitch_deg": state.get("pitch_deg"),
                "yaw_deg": state.get("yaw_deg"),
                "mode": state.get("mode"),
                "armed": state.get("armed"),
                "batt_v": state.get("batt_v"),
                "batt_pct": state.get("batt_pct"),
                "ekf_ready": state.get("ekf_ready"),
                "gps_fix_type": state.get("gps_fix_type"),
                "gps_sats": state.get("gps_sats"),
                "ned_n": state.get("ned_n"),
                "ned_e": state.get("ned_e"),
                "lat_deg": state.get("lat_deg"),
                "lon_deg": state.get("lon_deg"),
                "cpu_pct": None if cpu_pct is None else round(cpu_pct, 1),
            }
            compact = {k: v for k, v in payload.items() if v is not None or k in ("t", "link")}
            blob = json.dumps(compact).encode("utf-8")
            for dest in dests:
                try:
                    sock.sendto(blob, dest)
                except OSError as exc:
                    print(f"sendto {dest} failed: {exc}", file=sys.stderr, flush=True)
            if time.monotonic() - last_log > 10.0:
                last_log = time.monotonic()
                print(
                    "health: link={link} mode={mode} alt={alt} cpu={cpu}".format(
                        link=compact.get("link"),
                        mode=compact.get("mode"),
                        alt=compact.get("alt_m"),
                        cpu=compact.get("cpu_pct"),
                    ),
                    flush=True,
                )
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    sender = threading.Thread(target=send_loop, name="udp-send", daemon=True)
    sender.start()
    backoff = 1.0
    try:
        while True:
            try:
                source.connect(heartbeat_timeout=8.0)
                backoff = 1.0
                while True:
                    source.pump(budget_s=0.08)
                    time.sleep(0.01)
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                with source._lock:
                    source.link = "error"
                print(f"FC link error: {exc}; retry in {backoff:.0f}s", file=sys.stderr, flush=True)
                source.close()
                time.sleep(backoff)
                backoff = min(8.0, backoff * 1.5)
    except KeyboardInterrupt:
        print("Stopped.", flush=True)
    finally:
        stop.set()
        source.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="DroneLink MAVLink -> UDP telemetry")
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--dest-host", default=DEFAULT_DEST_HOST)
    parser.add_argument(
        "--dest-ports",
        default=DEFAULT_DEST_PORTS,
        help="comma-separated UDP ports (5701 preferred, 5601 legacy)",
    )
    parser.add_argument("--hz", type=float, default=DEFAULT_HZ)
    args = parser.parse_args()
    run_forever(args)


if __name__ == "__main__":
    main()
