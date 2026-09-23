#!/bin/bash
# 投放区单点测试 · L3 接近+对准单点：接口 + 真实 CV 感知 + 决策的 drop_zone_bench 场景
# （起飞→飞抵投放区→搜索→接近→投放，投满即 RTL）。**这一级会自主起飞并飞行**——飞手
# RC 全程待命、随时接管。
#
# 就是 run_mission_hardware.sh 挂 drop_zone_bench 场景（同一套真机节点/接线），单列一个
# 脚本是为对齐仿真侧 run_mission_sim_drop_zone_bench.sh、并把「这一级会飞」这件事讲清楚。
#
# ⚠ home 与航点：drop_zone_bench 的投放区中心/扫描航点来自 mission.real.yaml，真机 home
#   跟着解锁位置走（不像仿真有固定原点）。**首飞前务必按当天场地核对 mission.real.yaml 的
#   投放区坐标/高度**，确认从解锁点飞过去的航线安全。详见 docs/spec/field-test-runbook.md。
#
# 触发任务开始（另一个终端，栈停在 WAIT_TASK_START 后）：
#   ros2 topic pub -1 /cuadc_decision/sim/task_start_trigger std_msgs/msg/Empty "{}"
#
# Usage: ./scripts/run_field_l3_approach.sh [extra ros2 launch args]

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$REPO_ROOT/scripts/run_mission_hardware.sh" scenario:=drop_zone_bench "$@"
