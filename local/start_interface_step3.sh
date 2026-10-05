#!/bin/bash
# Step 3: start interface (no arm). Logs to /tmp/interface_bringup.log
set -e
cd /home/wly/cuadc_ws_worktrees/field-test
set +u
# shellcheck source=/dev/null
source scripts/activate_env.sh
set -u
nohup ros2 launch cuadc_interface interface_bringup.launch.py \
  connection:=udp:127.0.0.1:14550 \
  > /tmp/interface_bringup.log 2>&1 &
echo "launch_pid=$!"
sleep 2
pgrep -af interface_node || true
