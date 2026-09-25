#!/bin/bash
# 正式任务节点，真机目标：接口层 + 真实 CV 感知 + 决策，跟 run_mission_sim.sh
# 起完全相同的节点/launch 文件 (launch/mission_bringup.launch.py)，sim/真机之间
# 只差写死项：connection、profile、frame_source/预览，都不接受命令行覆盖，避免
# 手滑忘了传参连去了错误目标 / 取错帧源。
#
# connection 连底座 mav_fanout 的本地 MAVLink 端点 udp:127.0.0.1:14550，**不**直连
# /dev/ttyAMA0——串口由底座独占持有并扇出，任务链路直连串口会和底座抢占（见
# docs/spec/onboard-runtime.md、ADR-0008）。端点/协议若变要改这里。
#
# 真机 CV：frame_source=shm（读 camera_service 的 NV12，图不进 ROS）+ preview_enabled
# （标注推本地 5604，由 ground-monitor 的 cv-forward 转地面）。相机内参按 profile 从
# camera.real.yaml 读（见 ADR-0009），不再由脚本注入。
#
# Usage: ./scripts/run_mission_hardware.sh [extra ros2 launch args]

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"

LOG_FILTER="${LOG_FILTER:-}"

if [[ -n "$LOG_FILTER" ]]; then
    ros2 launch "$REPO_ROOT/launch/mission_bringup.launch.py" \
        connection:=udp:127.0.0.1:14550 profile:=real frame_source:=shm preview_enabled:=true "$@" \
        2>&1 | grep --line-buffered -E "$LOG_FILTER"
else
    exec ros2 launch "$REPO_ROOT/launch/mission_bringup.launch.py" \
        connection:=udp:127.0.0.1:14550 profile:=real frame_source:=shm preview_enabled:=true "$@"
fi
