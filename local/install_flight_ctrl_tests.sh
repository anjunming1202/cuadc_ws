#!/bin/bash
# Install bench scripts into ~/flight_ctrl_tests; stop interface_node only.
set -e
mkdir -p /home/wly/flight_ctrl_tests
cp /tmp/bench_motors.py /tmp/bench_drop.py /tmp/README.md /home/wly/flight_ctrl_tests/
chmod +x /home/wly/flight_ctrl_tests/bench_motors.py /home/wly/flight_ctrl_tests/bench_drop.py
sed -i 's/\r$//' /home/wly/flight_ctrl_tests/*.py /home/wly/flight_ctrl_tests/README.md
# Stop ROS interface so 14550 is free (do not touch fanout).
for pid in $(pgrep -f '/cuadc_interface/interface_node' || true); do
  kill "${pid}" || true
done
sleep 1
echo "=== flight_ctrl_tests ==="
ls -l /home/wly/flight_ctrl_tests
echo "=== remaining interface? ==="
pgrep -af '/cuadc_interface/interface_node' || echo none
echo "=== fanout ==="
pgrep -af mav_fanout.py || echo missing
