#!/bin/bash
# 压测台专用（sim-only）：mission_bringup 原样 + 相机→CV 图像延迟(保留拍照戳)，
# 见 mission_sim_delayed.launch.py。基线/生产跑 run_mission_sim.sh。
#
# Usage: ./scripts/run_mission_sim_delayed.sh <delay_ms> [extra ros2 launch args]

# No -u：ROS2 的 setup.bash 引用了没有默认值保护的变量（见 run_mission_sim.sh）。
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"

DELAY_MS="${1:-0.0}"
shift || true
exec ros2 launch cuadc_sim mission_sim_delayed.launch.py \
    "mission_bringup_path:=$REPO_ROOT/launch/mission_bringup.launch.py" \
    connection:=udp:127.0.0.1:14550 delay_ms:="$DELAY_MS" "$@"
