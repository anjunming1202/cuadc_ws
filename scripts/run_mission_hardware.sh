#!/bin/bash
# 正式任务节点，真机目标：接口层 + 真实 CV 感知 + 决策，跟
# run_mission_sim.sh 起的是完全相同的节点/launch 文件
# (launch/mission_bringup.launch.py)，唯一区别是 connection 写死指向真实
# 串口而不是 SITL——这是目前唯一允许在 sim/真机之间变化的参数，且不接受
# 命令行覆盖，避免哪天手滑忘了传参连去了错误目标。串口设备名/波特率如果
# 换了硬件要改这里。
#
# Usage: ./scripts/run_mission_hardware.sh [extra ros2 launch args]

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"

LOG_FILTER="${LOG_FILTER:-}"

if [[ -n "$LOG_FILTER" ]]; then
    ros2 launch "$REPO_ROOT/launch/mission_bringup.launch.py" connection:=/dev/ttyAMA0:57600 profile:=real "$@" \
        2>&1 | grep --line-buffered -E "$LOG_FILTER"
else
    exec ros2 launch "$REPO_ROOT/launch/mission_bringup.launch.py" connection:=/dev/ttyAMA0:57600 profile:=real "$@"
fi
