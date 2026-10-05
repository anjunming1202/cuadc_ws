#!/bin/bash
# Stop fanout, dump+probe 57600. Does not arm.
set -eu
pid=$(pgrep -f '/opt/cuadc-onboard/mav_fanout.py' || true)
echo "fanoutpid=${pid:-none}"
if [ -n "${pid}" ]; then
  kill ${pid} || true
  sleep 0.5
fi
/home/wly/dronelink-venv/bin/python3 /tmp/dump_uart_bytes.py /dev/ttyAMA0 57600 4
/home/wly/dronelink-venv/bin/python3 /tmp/probe_hb_once.py /dev/ttyAMA0 57600
echo "57600_exit=$?"
