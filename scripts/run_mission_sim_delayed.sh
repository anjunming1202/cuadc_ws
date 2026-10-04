#!/bin/bash
# 压测台专用（sim-only）：mission_bringup 原样 + 相机→CV 图像延迟(保留拍照戳)，
# 见 mission_sim_delayed.launch.py。基线/生产跑 run_mission_sim.sh。
#
# Usage: ./scripts/run_mission_sim_delayed.sh [delay_ms] [extra ros2 launch args]
#   delay_ms 省略时用扰动表 cuadc_sim/config/disturbances.yaml 的 image_latency_ms。

# No -u：ROS2 的 setup.bash 引用了没有默认值保护的变量（见 run_mission_sim.sh）。
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"

DELAY_ARGS=()
if [[ "${1:-}" =~ ^[0-9.]+$ ]]; then
    DELAY_ARGS=("delay_ms:=$1")
    shift
fi
exec ros2 launch cuadc_sim mission_sim_delayed.launch.py \
    "mission_bringup_path:=$REPO_ROOT/launch/mission_bringup.launch.py" \
    connection:=udp:127.0.0.1:14550 "${DELAY_ARGS[@]}" "$@"
