#!/bin/bash
# 全任务航线测试（仿真）：连 SITL + 跑 interface_route_test，节点代飞手解锁。先起 run_gazebo_demo.sh（整场地世界）。
# 起飞 → 投放区两个假目标（上方 3m → 降 1.4m 悬停 3s，不投放）→ 侦察区巡视航点（5m）→ RTL 降落。
# 与任务算法完全解耦：不起决策/感知，航线在节点参数里（见节点 docstring）。
#
# ⚠ 安全：飞机会自主飞到起飞点机头前方约 60m。飞手 RC 全程待命、随时接管；场地不够长用
#   --ros-args -p stub_targets:=... -p recon_waypoints:=... 改短。录制（onboard_record）另开。
# 放行：另一终端 ros2 topic pub -t 5 -r 2 /cuadc_flight_test/start std_msgs/msg/Empty "{}"，
#   之后飞手在 STABILIZE 用 RC 解锁（同飞行动作合集）。
#
# Usage: ./scripts/run_route_test_sim.sh [--ros-args -p name:=value ...]
set -eo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"
ros2 launch cuadc_interface interface_bringup.launch.py connection:=udp:127.0.0.1:14550 &
IFACE_PID=$!
_iface_sig='interface_node --ros-args -r __node:=cuadc_interface'
cleanup() { kill "$IFACE_PID" 2>/dev/null || true; pkill -INT -f "$_iface_sig" 2>/dev/null || true; sleep 2; pkill -9 -f "$_iface_sig" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
sleep 4
ros2 run cuadc_interface interface_route_test --ros-args -r __node:=interface_route_test -p emulate_pilot_arm:=true -- "$@"
