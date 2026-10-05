#!/bin/bash
# Stop fanout by exact pid, then probe ttyAMA0 at 115200. Does not arm.
set -eu
pid=$(pgrep -f '/opt/cuadc-onboard/mav_fanout.py' || true)
echo "fanoutpid=${pid:-none}"
if [ -n "${pid}" ]; then
  kill ${pid} || true
  sleep 0.5
fi
/home/wly/dronelink-venv/bin/python3 /tmp/probe_hb_once.py /dev/ttyAMA0 115200
echo "115200_exit=$?"
