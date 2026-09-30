#!/bin/bash
# ARM + RC 接管检查（真机，**每次飞行测试前必做**；也是比赛前给裁判演示 RC 能 disarm/夺控的环节）。
# 起接口层连 fanout + 跑 interface_arm_test：Pi arm → 给 RC 一个窗口 disarm/夺控 → 否则 Pi 自动 disarm。
# 验证 ① 树莓派能否 arm(最小控制打通)  ② RC 能否 disarm/接管(桨转前必须证明人能停下)。
#
# ⚠ 安全：arm 会让电机转(有桨则桨转)。按队内 SOP 决定是否拆桨——只验 RC disarm 信号链路用拆桨最安全;
#   要给裁判看电机真停再按其要求带桨并做好约束。飞手 RC 全程在手。
#   require_rc_disarm 真机默认 true：窗口内没看到 RC disarm 就判 FAIL,必须重做。
#
# 前置：底座在跑(mav_fanout 出 udp:127.0.0.1:14550)。无 GPS 的室内 bench 可 -p mode:=STABILIZE。
#
# Usage: ./scripts/run_arm_check_real.sh [extra ros2 run args]
#
# 放行闸门:节点起来停在 arm 前(桨不转),确认安全后另一 SSH 发
#   ros2 topic pub -1 /cuadc_flight_test/start std_msgs/msg/Empty "{}"
# 才 arm。

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
