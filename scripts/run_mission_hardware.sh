#!/bin/bash
# 正式任务节点，真机目标：接口层 + 真实 CV 感知 + 决策，跟 run_mission_sim.sh
# 起同一份 launch (launch/mission_bringup.launch.py)，差别是两处写死项：
# connection 指真实串口而非 SITL；frame_source=shm（真机从 camera_service 的
# NV12 shm 取帧，图不进 ROS），仿真是 ros（订 Gazebo 的 /camera/image_raw）。
# 都不接受命令行覆盖，避免手滑连错目标 / 取错帧源。串口/波特率换硬件要改这里。
#
# Usage: ./scripts/run_mission_hardware.sh [extra ros2 launch args]

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"

LOG_FILTER="${LOG_FILTER:-}"

if [[ -n "$LOG_FILTER" ]]; then
    ros2 launch "$REPO_ROOT/launch/mission_bringup.launch.py" connection:=/dev/ttyAMA0:57600 profile:=real frame_source:=shm "$@" \
        2>&1 | grep --line-buffered -E "$LOG_FILTER"
else
    exec ros2 launch "$REPO_ROOT/launch/mission_bringup.launch.py" connection:=/dev/ttyAMA0:57600 profile:=real frame_source:=shm "$@"
fi
