#!/bin/bash
# 飞行动作合集（仿真）：起接口层连 SITL + 跑 interface_flight_suite_test，
# 一次连续飞行走 起飞→定yaw方框→高度扫描→速度扫描→RTL 降落，打目标/实测/误差汇总表。
# sim 跑一遍给基线、验证指令/时序逻辑；real 换 run_flight_suite_real.sh 量精度，同格式对比。
#
# 前置：另一个终端已起 Gazebo+SITL，MAVLink 出 udp:127.0.0.1:14550。
#
# 选段跑（电量/时间）：追加 --ros-args -p phases:=box（只方框）等，见节点 docstring。
#
# Usage: ./scripts/run_flight_suite_sim.sh [extra ros2 run args]

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"

ros2 launch cuadc_interface interface_bringup.launch.py connection:=udp:127.0.0.1:14550 &
IFACE_PID=$!
# ros2 launch 的信号传播不可靠(SIGTERM/SIGINT 到父进程会留下孤儿 interface_node)，
# 按节点签名直接收:先 SIGINT 优雅关、再 SIGKILL 兜底。现场只应有这一个 cuadc_interface 节点。
_iface_sig='interface_node --ros-args -r __node:=cuadc_interface'
cleanup() { kill "$IFACE_PID" 2>/dev/null || true; pkill -INT -f "$_iface_sig" 2>/dev/null || true; sleep 2; pkill -9 -f "$_iface_sig" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

sleep 4
ros2 run cuadc_interface interface_flight_suite_test "$@"
