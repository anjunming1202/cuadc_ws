#!/bin/bash
# 飞行动作合集（真机）：起接口层连底座 fanout + 跑 interface_flight_suite_test，
# 一次连续飞行走 起飞→定yaw方框→高度扫描→速度扫描→RTL 降落，量实机指令/传感器精度。
#
# ⚠ 安全：飞机会自主飞完整段机动。**飞手 RC 全程待命、随时接管**。首飞前：
#   - 按场地核对水平余量：方框边长 box_leg_m（默认5m）、速度段单向 vel_leg_distance_m
#     （默认3.5m）都要在场地内，必要时用 --ros-args -p 调小；
#   - 高度扫描含 1m（贴近地效），起飞点上方及四周净空；
#   - 遥测/图传（cuadc_groundstation）+ recording（onboard_record）解耦另开，本脚本不启停。
#
# 前置：底座在跑（mav_fanout 出 udp:127.0.0.1:14550）。连 fanout 而非直连串口
# （见 docs/spec/onboard-runtime.md、ADR-0008）。
#
# 选段跑：追加 --ros-args -p phases:=box 等（takeoff/land 恒有），见节点 docstring。
#
# Usage: ./scripts/run_flight_suite_real.sh [extra ros2 run args]

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
