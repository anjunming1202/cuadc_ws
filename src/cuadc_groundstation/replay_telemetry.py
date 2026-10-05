#!/usr/bin/env python3
"""机载 rosbag/mcap → DroneLink 遥测 JSONL，并可 UDP 回放到 viewer。

从 zip/mcap 抽出 local_position / mode / 姿态 / GPS 等，写成与 telem_ground
相同字段的 jsonl；replay 时按时间戳发到 127.0.0.1:5701。
"""
from __future__ import annotations

import argparse
import io
import json
import math
import socket
import sys
import time
import zipfile
from pathlib import Path

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

# 话题 → 飞控字段（local_position 的 xyz 就是 MAVLink NED）
_T_POS = "/cuadc_interface/telemetry/local_position"
_T_MODE = "/cuadc_interface/status/mode"
_T_ATT = "/cuadc_interface/telemetry/attitude"
_T_GPS = "/cuadc_interface/telemetry/global_position"
_T_GPSST = "/cuadc_interface/telemetry/gps_status"
_T_BATT = "/cuadc_interface/telemetry/battery"
_T_EKF = "/cuadc_interface/telemetry/ekf_status"
_T_ARM = "/cuadc_interface/status/armed"


def _yaw_pitch_roll_deg(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """与 interface_node._quaternion_from_euler 互逆，输出度。"""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny, cosy)) % 360.0
    return yaw, pitch, roll


def _open_mcap(src: Path) -> io.BytesIO:
    if src.suffix.lower() == ".zip":
        zf = zipfile.ZipFile(src)
        names = [n for n in zf.namelist() if n.endswith(".mcap")]
        if not names:
            raise SystemExit(f"no .mcap in {src}")
        return io.BytesIO(zf.read(names[0]))
    return io.BytesIO(src.read_bytes())


def bag_to_rows(src: Path) -> list[dict]:
    """按 local_position 节拍打点，叠上最近的 mode/姿态/GPS。"""
    buf = _open_mcap(src)
    latest: dict = {}
    rows: list[dict] = []
    factory = DecoderFactory()
    reader = make_reader(buf)
    t0_ns: int | None = None
    wanted = {_T_POS, _T_MODE, _T_ATT, _T_GPS, _T_GPSST, _T_BATT, _T_EKF, _T_ARM}
    for schema, channel, message in reader.iter_messages():
        if channel.topic not in wanted:
            continue
        decoder = factory.decoder_for(channel.message_encoding, schema)
        if decoder is None:
            continue
        ros_msg = decoder(message.data)
        topic = channel.topic
        log_ns = int(message.log_time)
        if t0_ns is None:
            t0_ns = log_ns
        rel = (log_ns - t0_ns) / 1e9
        if topic == _T_MODE:
            latest["mode"] = str(ros_msg.data)
        elif topic == _T_ARM:
            latest["armed"] = bool(ros_msg.data)
        elif topic == _T_ATT:
            yaw, pitch, roll = _yaw_pitch_roll_deg(
                ros_msg.orientation.x, ros_msg.orientation.y,
                ros_msg.orientation.z, ros_msg.orientation.w)
            latest["yaw_deg"] = round(yaw, 2)
            latest["pitch_deg"] = round(pitch, 2)
            latest["roll_deg"] = round(roll, 2)
        elif topic == _T_GPS:
            latest["lat_deg"] = float(ros_msg.latitude)
            latest["lon_deg"] = float(ros_msg.longitude)
        elif topic == _T_GPSST:
            latest["gps_fix_type"] = int(ros_msg.fix_type)
            latest["gps_sats"] = int(ros_msg.satellites_visible)
        elif topic == _T_BATT:
            latest["batt_v"] = round(float(ros_msg.voltage), 2)
            pct = float(ros_msg.percentage)
            if math.isfinite(pct):
                latest["batt_pct"] = round(pct * 100.0 if pct <= 1.0 else pct, 1)
        elif topic == _T_EKF:
            latest["ekf_ready"] = bool(ros_msg.healthy)
        elif topic == _T_POS:
            p = ros_msg.pose.position
            row = {
                "t": rel,  # 相对袋起始秒；replay 用
                "ned_n": float(p.x),
                "ned_e": float(p.y),
                "alt_m": float(-p.z),  # NED z 朝下
                "link": "replay",
            }
            row.update(latest)
            rows.append(row)
    return rows


def write_jsonl(rows: list[dict], dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    with dest.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")


def summarize(rows: list[dict]) -> None:
    if not rows:
        print("no local_position samples", file=sys.stderr)
        return
    modes = []
    for r in rows:
        m = r.get("mode")
        if m and (not modes or modes[-1][1] != m):
            modes.append((r["t"], m))
    alts = [r["alt_m"] for r in rows]
    print(f"samples={len(rows)} duration={rows[-1]['t']:.1f}s")
    print(f"alt_m min={min(alts):.2f} max={max(alts):.2f}")
    print("mode changes:")
    for t, m in modes:
        print(f"  t={t:7.1f}s  {m}")


def replay(path: Path, host: str, port: int, speed: float) -> None:
    rows = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    if not rows:
        raise SystemExit(f"empty {path}")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (host, port)
    print(f"replay {len(rows)} pts -> {host}:{port} speed={speed}  (Ctrl+C stop)")
    t_wall0 = time.monotonic()
    try:
        for row in rows:
            target = row["t"] / max(speed, 0.01)
            delay = target - (time.monotonic() - t_wall0)
            if delay > 0:
                time.sleep(delay)
            pkt = dict(row)
            pkt["t"] = time.time()  # viewer 高度窗用墙钟
            sock.sendto(json.dumps(pkt).encode("utf-8"), dest)
    except KeyboardInterrupt:
        print("\nstopped")


def main() -> None:
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser(description="Convert/replay flight bag for DroneLink")
    sub = p.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("convert", help="zip/mcap → jsonl")
    c.add_argument("src", type=Path)
    c.add_argument("-o", "--out", type=Path, default=None)

    r = sub.add_parser("replay", help="jsonl → UDP 5701")
    r.add_argument("jsonl", type=Path)
    r.add_argument("--host", default="127.0.0.1")
    r.add_argument("--port", type=int, default=5701)
    r.add_argument("--speed", type=float, default=1.0, help=">1 加速")

    args = p.parse_args()
    if args.cmd == "convert":
        rows = bag_to_rows(args.src)
        out = args.out or (here / "replays" / (args.src.stem + ".jsonl"))
        write_jsonl(rows, out)
        summarize(rows)
        print(f"wrote {out}")
        print(f"replay: python replay_telemetry.py replay {out}")
    else:
        replay(args.jsonl, args.host, args.port, args.speed)


if __name__ == "__main__":
    main()
