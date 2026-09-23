#!/bin/bash
# 正式任务节点，真机目标：接口层 + 真实 CV 感知 + 决策，跟 run_mission_sim.sh
# 起的是完全相同的节点/launch 文件 (launch/mission_bringup.launch.py)，sim/真机
# 之间只差三处写死项：connection、profile、真实 CV 接线，都不接受命令行覆盖，避免
# 哪天手滑忘了传参连去了错误目标 / 用错内参。
#
# connection 连底座 mav_fanout 的本地 MAVLink 端点 udp:127.0.0.1:14550，**不**直连
# /dev/ttyAMA0——串口由底座独占持有并扇出，任务链路直连串口会和底座抢占（见
# docs/spec/onboard-runtime.md、ADR-0008）。端点/协议若变要改这里。
#
# 真实 CV 接线（frame_source=shm + 真实内参 + 预览）来自 real_camera_intrinsics.sh
# 单一真源，见该文件。
#
# Usage: ./scripts/run_mission_hardware.sh [extra ros2 launch args]

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"
source "$REPO_ROOT/scripts/real_camera_intrinsics.sh"   # -> REAL_CV_LAUNCH_ARGS

LOG_FILTER="${LOG_FILTER:-}"

if [[ -n "$LOG_FILTER" ]]; then
    ros2 launch "$REPO_ROOT/launch/mission_bringup.launch.py" \
        connection:=udp:127.0.0.1:14550 profile:=real "${REAL_CV_LAUNCH_ARGS[@]}" "$@" \
        2>&1 | grep --line-buffered -E "$LOG_FILTER"
else
    exec ros2 launch "$REPO_ROOT/launch/mission_bringup.launch.py" \
        connection:=udp:127.0.0.1:14550 profile:=real "${REAL_CV_LAUNCH_ARGS[@]}" "$@"
fi
