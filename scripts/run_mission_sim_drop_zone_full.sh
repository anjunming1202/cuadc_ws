#!/bin/bash
# 投放区全流程测试任务节点（仿真）：scenario:=drop_zone_full —— 整区搜索、最小筒优先、投满 2 次即 RTL，
# 与正式任务同一套 SEARCH/选目标逻辑。配合 run_gazebo_drop_zone_bench.sh 的近投放区出生世界用（出生点
# 写在 drop_zone_full.sim.yaml），真机对应 run_drop_zone_full_real.sh。
#
# Usage: ./scripts/run_mission_sim_drop_zone_full.sh [extra ros2 launch args]
#
# 两个脚本都起来后，另开终端触发任务：
#   ros2 topic pub -t 5 -r 2 /cuadc_decision/sim/task_start_trigger std_msgs/msg/Empty "{}"
set -eo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$REPO_ROOT/scripts/run_mission_sim.sh" scenario:=drop_zone_full "$@"
