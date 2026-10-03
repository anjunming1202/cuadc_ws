#!/usr/bin/env python3
"""cuadc_onboard 传输底座：飞控串口 ↔ 多个本地 UDP 端点的原始 MAVLink 扇出。

串口侧用 **mavutil**（与 dronelink-telemetry 同一套经过验证的做法）：
`mavlink_connection` 打开串口（pyserial 默认拉高 DTR/RTS——这块飞控要拉高才发）、
等首个心跳、再请求各数据流并周期性重请求，把飞控"喂活"。读到消息后把**原始
字节** `get_msgbuf()` 主动推给每个本地 UDP 端点；反向把消费者字节写回串口。

消费者用 `udpin:127.0.0.1:<port>`（绑定即收，语义等同 mavproxy `--out`），
即使"先读后写"的消费者（如 cuadc_interface）也立刻收到稳定流。

为什么不用 mavlink-router / 裸 pyserial keepalive：见 ADR-0004（DTR/RTS 线状态、
以及这块飞控要有人用 mavutil 那套请求序列持续喂才稳定发）。

这块飞控 serial 关再开会把 TELEM 卡死（要飞控断电才恢复），所以心跳超时和
循环异常都**不 close 串口**；只在设备 open 失败时重试。
"""
from __future__ import annotations

import argparse
import select
import socket
import sys
import time

from pymavlink import mavutil

_STREAMS = (
    mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
)


def request_streams(m, rate: int) -> None:
    sysid = m.target_system or 1
    compid = m.target_component or 1
    m.mav.request_data_stream_send(
        sysid, compid, mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)
    for s in _STREAMS:
        m.mav.request_data_stream_send(sysid, compid, s, rate, 1)


def main() -> None:
    ap = argparse.ArgumentParser(description="MAVLink serial<->UDP fanout (mavutil)")
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
    print(f"fanout(mavutil): {args.serial}@{args.baud} -> udp {dests}", flush=True)

    m = None
    backoff = 1.0
    last_req = 0.0
    last_wait_log = 0.0
    got_hb = False
    while True:
        if m is None:
            try:
                m = mavutil.mavlink_connection(
                    args.serial, baud=args.baud,
                    source_system=250, source_component=190)
            except Exception as exc:
                print(f"open failed: {exc}; retry {backoff:.1f}s", file=sys.stderr, flush=True)
                time.sleep(backoff)
                backoff = min(10.0, backoff * 1.5)
                continue
            got_hb = False
            backoff = 1.0
            request_streams(m, args.rate)  # sysid 默认 1；未解锁也会发心跳
            last_req = time.monotonic()
            last_wait_log = last_req
            print("serial held open; waiting for heartbeat (will not reopen)", flush=True)

        try:
            now = time.monotonic()
            if now - last_req > 2.0:  # 持续重请求，喂活数据流
                request_streams(m, args.rate)
                last_req = now
            if not got_hb and now - last_wait_log > 30.0:
                print("still no heartbeat; keeping serial open", file=sys.stderr, flush=True)
                last_wait_log = now

            serial_fd = getattr(m, "fd", None)
            rlist = [u.fileno() for u in socks]
            if serial_fd is not None:
                rlist.append(serial_fd)
            ready, _, _ = select.select(rlist, [], [], 0.5)

            # 消费者 -> 串口
            for u in socks:
                if u.fileno() in ready:
                    try:
                        data, _ = u.recvfrom(65535)
                        m.write(data)
                    except OSError:
                        pass

            # 串口 -> 消费者（读解析后转发原始字节；容错 pymavlink 偶发 quirk）
            if serial_fd is None or serial_fd in ready:
                for _ in range(500):
                    try:
                        msg = m.recv_match(blocking=False)
                    except Exception:
                        continue  # 跳过个别消息的 pymavlink post_message quirk
                    if msg is None:
                        break
                    if msg.get_type() == "BAD_DATA":
                        continue
                    if (not got_hb) and msg.get_type() == "HEARTBEAT":
                        got_hb = True
                        print(
                            f"heartbeat sys={m.target_system} "
                            f"comp={m.target_component}",
                            flush=True,
                        )
                    buf = msg.get_msgbuf()
                    for u, dest in zip(socks, dests):
                        try:
                            u.sendto(buf, dest)
                        except OSError:
                            pass
        except Exception as exc:
            print(f"loop error: {exc}; keeping serial", file=sys.stderr, flush=True)
            time.sleep(0.5)


if __name__ == "__main__":
    main()
