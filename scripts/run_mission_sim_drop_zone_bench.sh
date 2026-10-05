#!/bin/bash
# 投放区单点测试任务节点：scenario:=drop_zone_bench —— 从投放前一个 state 起跑
# （起飞→飞抵投放区→搜索→接近→投放），投满即 RTL（phases=[drop]），并放宽 SEARCH
# 选目标门（看到候选就投），全部由场景配置表达、mission_logic 无 test-only 分支，见
# docs/spec/mission-config-layer.md。配合 run_gazebo_drop_zone_bench.sh 用，仅仿真可用
# ——出生点搬近投放区依赖仿真里 home 原点固定不变，实机 home 跟着解锁位置走，不能直接
# 照搬，实机单点测试到时候另外写。真实 CV 感知（不是桩），跟正式任务同一套节点，见
# run_mission_sim.sh。
#
# Usage: ./scripts/run_mission_sim_drop_zone_bench.sh [extra ros2 launch args]
#
# 两个脚本都起来后，另开终端触发任务：
#   ros2 topic pub -t 5 -r 2 /cuadc_decision/sim/task_start_trigger std_msgs/msg/Empty "{}"

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$REPO_ROOT/scripts/run_mission_sim.sh" scenario:=drop_zone_bench "$@"
