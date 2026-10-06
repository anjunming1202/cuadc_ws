#!/bin/bash
# 投放区全流程测试（真机）：接口 + 真实 CV 感知 + 决策的 drop_zone_full 场景（起飞→飞抵投放区→整区
# 搜索→最小筒优先→接近投放，投满 2 次或超时放弃即 RTL）。**这一级会自主起飞并飞行**——飞手 RC 全程
# 待命、随时接管。
#
# 就是 run_mission_real.sh 挂 drop_zone_full 场景（同一套真机节点/接线）。投放区按规则书 5m×8m，
# 近边在起飞时机头正前方 2m（drop_zone_full.real.yaml）：飞机放在起飞点、机头朝投放区，筒摆在
# x∈[2,7]、y∈[-4,4] 内（x 向前、y 向右，米）。详见 docs/spec/field-test-runbook.md。
#
# 触发任务开始（另一个终端，栈停在 WAIT_TASK_START 后）：
#   ros2 topic pub -t 5 -r 2 /cuadc_decision/sim/task_start_trigger std_msgs/msg/Empty "{}"
#
# Usage: ./scripts/run_drop_zone_full_real.sh [extra ros2 launch args, e.g. approach:=... approach_variant:=...]

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$REPO_ROOT/scripts/run_mission_real.sh" scenario:=drop_zone_full "$@"
