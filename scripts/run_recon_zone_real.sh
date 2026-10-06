#!/bin/bash
# 侦察区单点测试（真机）：接口 + 真实 CV 感知 + 决策的 recon_zone_bench 场景（起飞→飞抵侦察区→
# 覆盖侦察→RTL）。**这一级会自主起飞并飞行**——飞手 RC 全程待命、随时接管。
#
# 就是 run_mission_real.sh 挂 recon_zone_bench 场景（同一套真机节点/接线），单列一个脚本是为
# 对齐仿真侧 run_mission_sim_recon_zone_bench.sh、并把「这一级会飞」这件事讲清楚。
#
# ⚠ 侦察区中心来自 recon_zone_bench.real.yaml（起飞时机头前方，按当天摆筒改），扫描样式/高度
#   来自 mission.real.yaml。**首飞前务必核对**，确认从解锁点飞过去的航线安全。
#
# 触发任务开始（另一个终端，栈停在 WAIT_TASK_START 后）：
#   ros2 topic pub -t 5 -r 2 /cuadc_decision/sim/task_start_trigger std_msgs/msg/Empty "{}"
#
# Usage: ./scripts/run_recon_zone_real.sh [extra ros2 launch args]

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$REPO_ROOT/scripts/run_mission_real.sh" scenario:=recon_zone_bench "$@"
