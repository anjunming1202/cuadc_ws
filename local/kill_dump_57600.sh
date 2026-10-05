#!/bin/bash
# Stop fanout, dump raw UART at 57600. Does not arm.
set -eu
pid=$(pgrep -f '/opt/cuadc-onboard/mav_fanout.py' || true)
echo "fanoutpid=${pid:-none}"
if [ -n "${pid}" ]; then
  kill ${pid} || true
  sleep 0.5
fi
/home/wly/dronelink-venv/bin/python3 /tmp/dump_uart_bytes.py /dev/ttyAMA0 57600 3
echo "57600_dump_done"
