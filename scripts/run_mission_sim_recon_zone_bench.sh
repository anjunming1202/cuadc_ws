#!/bin/bash
# 侦察区单点测试任务节点：scenario:=recon_zone_bench —— 起飞后直接飞抵侦察区→覆盖侦察→RTL
# （phases=[recon]），由场景配置表达、mission_logic 无 test-only 分支，见
# docs/spec/mission-config-layer.md。配合 run_gazebo_recon_zone_bench.sh 用；sim 的近侦察区
# 出生点写在 recon_zone_bench.sim.yaml，真机对应 run_recon_zone_real.sh（recon_zone_bench.real.yaml）。
# 真实 CV 感知（不是桩），跟正式任务同一套节点，见 run_mission_sim.sh。
#
# Usage: ./scripts/run_mission_sim_recon_zone_bench.sh [extra ros2 launch args]
#
# 两个脚本都起来后，另开终端触发任务：
#   ros2 topic pub -t 5 -r 2 /cuadc_decision/sim/task_start_trigger std_msgs/msg/Empty "{}"

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$REPO_ROOT/scripts/run_mission_sim.sh" scenario:=recon_zone_bench "$@"
