#!/bin/bash
# ARM + RC 接管检查（真机，**每次飞行测试前必做**；也是比赛前给裁判演示 RC 能 disarm/夺控的环节）。
# 起接口层连 fanout + 跑 interface_arm_test：放行 → 飞手 RC 解锁 → 15s 窗口内飞手 RC disarm/夺控 →
# 否则飞控自动 disarm(Pi 兜底)。流程与实际飞行一致(解锁/上锁归飞手,Pi 不发 arm)。
# 验证 ① Pi 能看到飞手解锁(并核对是否 STABILIZE 挡)  ② RC 能否 disarm/接管(桨转前必须证明人能停下)。
# 只如实记录事件,不判 PASS/FAIL。
#
# ⚠ 安全：解锁会让电机转(有桨则桨转)。按队内 SOP 决定是否拆桨——只验 RC disarm 信号链路用拆桨最安全;
#   要给裁判看电机真停再按其要求带桨并做好约束。飞手 RC 全程在手。
#
# 前置：底座在跑(mav_fanout 出 udp:127.0.0.1:14550)。默认不等 EKF(室内无 GPS 也能做)。
#
# Usage: ./scripts/run_arm_check_real.sh [extra ros2 run args]
#
# 节点起来停在 READY(桨不转),确认安全后另一 SSH 发放行,之后飞手 RC 解锁:
#   ros2 topic pub -t 5 -r 2 /cuadc_flight_test/start std_msgs/msg/Empty "{}"

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
