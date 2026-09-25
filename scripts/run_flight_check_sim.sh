#!/bin/bash
# 起降 smoke（仿真）：起接口层连 SITL + 跑 interface_takeoff_land_test，
# 验证「解锁→起飞→悬停→降落→上锁」最短链路。不含感知/决策，飞机只上下动。
#
# 前置：另一个终端已起 Gazebo+SITL（run_gazebo_demo.sh / run_gazebo_drop_zone_bench.sh），
# SITL 的 MAVLink 出到 udp:127.0.0.1:14550。本脚本起接口层连它，跑完自动收尾。
#
# 真机等价：run_flight_check_real.sh。同一个测试节点，只连接目标不同。
#
# Usage: ./scripts/run_flight_check_sim.sh [extra ros2 run args, e.g. --ros-args -p takeoff_altitude_m:=3.0]

# No -u: ROS2 setup.bash 引用未定义变量，nounset 会破坏 source。
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

# 接口层连上 SITL + 首帧遥测就绪需一点时间；测试节点自身也会等服务/EKF，这里只给个起步余量。
sleep 4
ros2 run cuadc_interface interface_takeoff_land_test "$@"
