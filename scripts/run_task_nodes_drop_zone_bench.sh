#!/bin/bash
# 投放区单点测试任务节点：stop_after_drop 默认 true，投完两次直接 RTL。
# 配合 run_gazebo_drop_zone_bench.sh 用，仅仿真可用——出生点搬近投放区
# 这个技巧依赖仿真里 home 原点固定不变的特性，实机上 home 真的跟着解锁
# 位置走，不能直接照搬，实机单点测试到时候另外写。
#
# Usage: ./scripts/run_task_nodes_drop_zone_bench.sh [extra ros2 launch args]
#
# 两个脚本都起来后，另开终端触发任务：
#   ros2 topic pub --once /cuadc_decision/sim/task_start_trigger std_msgs/msg/Empty "{}"

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$REPO_ROOT/scripts/run_task_nodes.sh" stop_after_drop:=true "$@"
