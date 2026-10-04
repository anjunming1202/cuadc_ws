#!/bin/bash
# 起降 smoke（真机）：起接口层连底座 fanout + 跑 interface_takeoff_land_test，
# 验证树莓派能否指挥飞控完成「解锁→起飞→悬停→降落→上锁」。飞机只上下动，不含感知/决策。
#
# ⚠ 安全：飞机会真起飞。**飞手 RC 全程待命、随时接管**；首飞前确认起飞点上方净空。
# 遥测/图传（cuadc_groundstation）与 recording（onboard_record）是解耦的独立组件，
# 按需另开——本脚本只驱动飞行，不启停它们。
#
# 前置：底座在跑（mav_fanout 出 udp:127.0.0.1:14550）。连 fanout 而非直连串口——
# 串口由底座独占扇出（见 docs/spec/onboard-runtime.md、ADR-0008）。
#
# Usage: ./scripts/run_flight_check_real.sh [extra ros2 run args, e.g. --ros-args -p takeoff_altitude_m:=3.0]
#
# 起飞条件(固定顺序):节点起来停在 READY → 另一 SSH 发放行
#   ros2 topic pub -1 /cuadc_flight_test/start std_msgs/msg/Empty "{}"
# → 飞手模式开关在 STABILIZE 挡、用 RC 解锁 → Pi 1s 内切 GUIDED 起飞。Pi 不发 arm。
# 飞行中飞手拨模式开关/RC 上锁 = 夺控,Pi 立即停发指令退出。

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
ros2 run cuadc_interface interface_takeoff_land_test "$@"
