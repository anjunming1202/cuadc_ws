#!/bin/bash
# 压测台专用（sim-only）：走 mission_sim_delayed.launch.py，在感知↔决策间插延迟节点
# 建模检测时延。只有 search_stress.py 的时延档用它；基线/生产跑 run_mission_sim.sh。
#
# Usage: ./scripts/run_mission_sim_delayed.sh <delay_ms>

# No -u：ROS2 的 setup.bash 引用了没有默认值保护的变量（见 run_mission_sim.sh）。
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"

DELAY_MS="${1:-0.0}"
exec ros2 launch cuadc_sim mission_sim_delayed.launch.py \
    connection:=udp:127.0.0.1:14550 profile:=sim delay_ms:="$DELAY_MS"
