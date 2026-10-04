#!/bin/bash
# Read ROS telemetry once. Does not call Arm.
set -e
cd /home/wly/cuadc_ws_worktrees/field-test
set +u
# shellcheck source=/dev/null
source scripts/activate_env.sh
set -u
echo "=== node ==="
ros2 node list 2>/dev/null | grep -i interface || true
echo "=== armed ==="
timeout 8 ros2 topic echo /cuadc_interface/status/armed --once || echo "armed: TIMEOUT"
echo "=== mode ==="
timeout 8 ros2 topic echo /cuadc_interface/status/mode --once || echo "mode: TIMEOUT"
echo "=== ekf ==="
timeout 8 ros2 topic echo /cuadc_interface/telemetry/ekf_status --once || echo "ekf: TIMEOUT"
echo "=== connected ==="
timeout 8 ros2 topic echo /cuadc_interface/status/connected --once || echo "connected: TIMEOUT"
