#!/bin/bash
# ARM 检查（仿真）：起接口层连 SITL + 跑 interface_arm_test，验 Pi arm→观察窗口→出口链路。
# sim 无 RC，观察窗口内不会有 RC disarm/夺控，一般走「窗口结束→飞控自动 disarm / Pi 兜底」出口；
# RC disarm/夺控只能真机验，见 run_arm_check_real.sh。用于上真机前先在仿真确认节点逻辑。
#
# 前置：另一个终端已起 Gazebo+SITL，MAVLink 出 udp:127.0.0.1:14550。
#
# Usage: ./scripts/run_arm_check_sim.sh [extra ros2 run args]
#
# 放行闸门:节点起来停在 arm 前,另一 SSH 发
#   ros2 topic pub -1 /cuadc_flight_test/start std_msgs/msg/Empty "{}"
# 才 arm(sim 可 -p wait_for_start:=false 跳过)。

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
ros2 run cuadc_interface interface_arm_test "$@"
