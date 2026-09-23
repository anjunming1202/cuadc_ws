#!/bin/bash
# 压测台专用（sim-only）：跟 run_mission_sim.sh 一样起任务三节点，但走
# mission_sim_delayed.launch.py——在感知↔决策之间插一个延迟节点，把检测延迟
# 指定毫秒再交给决策，用来在仿真里建模真实检测流水线时延。只有 search_stress.py
# 的时延档会用它；生产/基线仿真跑 run_mission_sim.sh（那条链零延迟、不含延迟节点）。
# 见 docs/spec/sim-stress-testbed.md §3.2。
#
# Usage: ./scripts/run_mission_sim_delayed.sh <delay_ms>

# No -u：ROS2 的 setup.bash 引用了没有默认值保护的变量（见 run_mission_sim.sh）。
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"

DELAY_MS="${1:-0.0}"
exec ros2 launch cuadc_sim mission_sim_delayed.launch.py \
    connection:=udp:127.0.0.1:14550 profile:=sim delay_ms:="$DELAY_MS"
