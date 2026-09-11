#!/bin/bash
# 桩感知任务节点，仿真目标：接口层 + 桩感知（地面真值）+ 决策
# (launch/mission_bringup_stub.launch.py)。只用于快速迭代决策层逻辑，
# **不是**正式任务/闭环验证用的脚本——正式的是 run_mission_sim.sh（真实
# CV 感知）。文件名/脚本名都直接带 stub 字样，不靠参数默认值区分。
#
# Usage: ./scripts/run_mission_stub_sim.sh [extra ros2 launch args]

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"

LOG_FILTER="${LOG_FILTER:-}"

if [[ -n "$LOG_FILTER" ]]; then
    ros2 launch "$REPO_ROOT/launch/mission_bringup_stub.launch.py" connection:=udp:127.0.0.1:14550 "$@" \
        2>&1 | grep --line-buffered -E "$LOG_FILTER"
else
    exec ros2 launch "$REPO_ROOT/launch/mission_bringup_stub.launch.py" connection:=udp:127.0.0.1:14550 "$@"
fi
