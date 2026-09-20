#!/bin/bash
# 正式任务节点，仿真目标：接口层 + 真实 CV 感知 + 决策
# (launch/mission_bringup.launch.py at repo root -- not owned by any single
# package since it composes nodes from three of them, see that file's
# docstring). Does NOT start the sim world itself (see run_gazebo_demo.sh)
# and does NOT trigger task start -- that stays a separate terminal by
# design, see README.md Quickstart.
#
# connection 写死指向本机 SITL，不接受命令行覆盖——真机跑
# run_mission_hardware.sh，桩感知跑 run_mission_stub_sim.sh，一个脚本只
# 对应一种场景，不靠传参切换，见 launch/mission_bringup.launch.py 文件头
# 的说明（之前那种"一个脚本+参数默认值决定行为"的设计就是因为默认值
# 悄悄选中了错误的选项，排查了很久才发现）。
#
# Usage: ./scripts/run_mission_sim.sh [extra ros2 launch args, e.g. stop_after_drop:=true]

# No -u (nounset): ROS2's own setup.bash files reference variables like
# AMENT_TRACE_SETUP_FILES without a default-value guard, assuming bash's
# normal "unset expands to empty" behavior -- nounset breaks sourcing them.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"

# 接口/感知/决策三个节点的日志会挤在同一个终端里。要只看某一类，改这个
# 正则（纯 shell 脚本，改完直接跑，不用 rebuild），或者不改文件、临时用
# 环境变量覆盖一次：LOG_FILTER='\[STATE\]' ./scripts/run_mission_sim.sh
#   LOG_FILTER='\[STATE\]'            只看状态切换
#   LOG_FILTER='\[STATE\]|\[PERF\]'   状态切换 + 耗时监控
# 留空 = 不过滤，全部日志都显示（默认）。
LOG_FILTER="${LOG_FILTER:-}"

if [[ -n "$LOG_FILTER" ]]; then
    ros2 launch "$REPO_ROOT/launch/mission_bringup.launch.py" connection:=udp:127.0.0.1:14550 profile:=sim "$@" \
        2>&1 | grep --line-buffered -E "$LOG_FILTER"
else
    exec ros2 launch "$REPO_ROOT/launch/mission_bringup.launch.py" connection:=udp:127.0.0.1:14550 profile:=sim "$@"
fi
