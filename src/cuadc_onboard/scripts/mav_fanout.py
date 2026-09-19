#!/usr/bin/env python3
"""cuadc_onboard 传输底座：飞控串口 ↔ 多个本地 UDP 端点的原始 MAVLink 扇出。

独占 /dev/ttyAMA0，把飞控原始 MAVLink 字节**主动推**到每个本地 UDP 端点；
反向把消费者发来的字节写回串口。不解析、不重编码，所有消息类型原样透传。

**主动 keepalive**：扇出自己周期性给飞控发心跳 + 请求数据流，把飞控"喂活"
（这块 ArduPilot 的数据流要有人持续请求才会一直发）。这样消费者哪怕是纯
"先读后写"（如 cuadc_interface）也能立刻收到稳定的遥测流，不必自己先请求。

消费者用 `udpin:127.0.0.1:<port>`（绑定该端口收）连一个端点，语义等同 mavproxy
`--out=udp:127.0.0.1:<port>`。

为什么不用 mavlink-router：这块飞控只有 DTR 或 RTS 拉高时才发（两条都低就哑）；
pyserial 打开串口默认把两条都拉高，mavlink-router 不拉 → 读不到。
"""
from __future__ import annotations

import argparse
import select
import socket
import sys
import time

import serial
from pymavlink import mavutil


def open_serial(dev: str, baud: int) -> serial.Serial:
    s = serial.Serial()
    s.port = dev
    s.baudrate = baud
    s.timeout = 0
    s.dtr = True   # 关键：这块飞控 DTR/RTS 都低才哑，拉高才发
    s.rts = True
    s.open()
    return s


def main() -> None:
    ap = argparse.ArgumentParser(description="Active MAVLink serial<->UDP fanout")
    ap.add_argument("--serial", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--udp", default="14550,14551",
                    help="逗号分隔本地 UDP 端口；消费者用 udpin:127.0.0.1:<port> 收")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--rate", type=int, default=10, help="向飞控请求数据流的速率 Hz")
    args = ap.parse_args()

    ports = [int(p) for p in args.udp.split(",") if p.strip()]
    dests = [(args.host, p) for p in ports]
    socks = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in ports]
    for u in socks:
        u.setblocking(False)
    print(f"fanout(active): {args.serial}@{args.baud} -> udp {dests}", flush=True)

    ser: serial.Serial | None = None
    mav = None
    backoff = 1.0
    last_req = 0.0
    while True:
        if ser is None:
            try:
                ser = open_serial(args.serial, args.baud)
                mav = mavutil.mavlink.MAVLink(ser, srcSystem=250, srcComponent=190)
                backoff = 1.0
                last_req = 0.0
                print(f"serial opened {args.serial}", flush=True)
            except Exception as exc:
                print(f"serial open failed: {exc}; retry {backoff:.1f}s",
                      file=sys.stderr, flush=True)
                time.sleep(backoff)
                backoff = min(10.0, backoff * 1.5)
                continue
        try:
            now = time.monotonic()
            if now - last_req > 2.0:  # keepalive：喂活飞控的数据流
                try:
                    mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                       mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    mav.request_data_stream_send(
                        1, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)
                except Exception as exc:
                    print(f"keepalive send error: {exc}", file=sys.stderr, flush=True)
                last_req = now

            rlist = [ser.fileno()] + [u.fileno() for u in socks]
            ready, _, _ = select.select(rlist, [], [], 0.5)
            for u in socks:
                if u.fileno() in ready:
                    try:
                        data, _ = u.recvfrom(65535)
                        ser.write(data)
                    except OSError:
                        pass
            if ser.fileno() in ready:
                data = ser.read(4096)
                if data:
                    for u, dest in zip(socks, dests):
                        try:
                            u.sendto(data, dest)
                        except OSError:
                            pass
        except Exception as exc:
            print(f"loop error: {exc}; reopening serial", file=sys.stderr, flush=True)
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            mav = None
            time.sleep(0.5)


if __name__ == "__main__":
    main()
