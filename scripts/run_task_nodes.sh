#!/bin/bash
# Convenience wrapper for the task-node stack: interface + perception +
# decision (launch/mission_bringup.launch.py at repo root -- not owned by
# any single package since it composes nodes from three of them, see that
# file's docstring). This is what runs on real hardware too -- the only
# thing that changes there is the `connection` arg (real serial port
# instead of SITL's UDP). Does NOT start the sim world itself (see
# run_gazebo_demo.sh/run_sitl_demo.sh) and does NOT trigger task start --
# that stays a separate terminal by design, see README.md Quickstart.
#
# Usage: ./scripts/run_task_nodes.sh [extra ros2 launch args]
#   ./scripts/run_task_nodes.sh                                   # sim (SITL UDP)
#   ./scripts/run_task_nodes.sh connection:=/dev/ttyACM0:57600     # real hardware

# No -u (nounset): ROS2's own setup.bash files reference variables like
# AMENT_TRACE_SETUP_FILES without a default-value guard, assuming bash's
# normal "unset expands to empty" behavior -- nounset breaks sourcing them.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"

# 接口/感知/决策三个节点的日志会挤在同一个终端里。要只看某一类，改这个
# 正则（纯 shell 脚本，改完直接跑，不用 rebuild），或者不改文件、临时用
# 环境变量覆盖一次：LOG_FILTER='\[STATE\]' ./scripts/run_task_nodes.sh
#   LOG_FILTER='\[STATE\]'            只看状态切换
#   LOG_FILTER='\[STATE\]|\[PERF\]'   状态切换 + 耗时监控
# 留空 = 不过滤，全部日志都显示（默认）。
LOG_FILTER="${LOG_FILTER:-}"

if [[ -n "$LOG_FILTER" ]]; then
    ros2 launch "$REPO_ROOT/launch/mission_bringup.launch.py" "$@" 2>&1 | grep --line-buffered -E "$LOG_FILTER"
else
    exec ros2 launch "$REPO_ROOT/launch/mission_bringup.launch.py" "$@"
fi
