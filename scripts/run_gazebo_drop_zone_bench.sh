#!/bin/bash
# 投放区单点测试仿真世界：飞机出生在投放区旁边，不用飞完整段转场。仅
# 仿真可用（world 文件是 Gazebo 专属的），实机单点测试到时候另外写。
# 配合 run_mission_sim_drop_zone_bench.sh 在另一个终端用。
#
# Usage: ./scripts/run_gazebo_drop_zone_bench.sh [extra ros2 launch args]
#   ./scripts/run_gazebo_drop_zone_bench.sh                 # normal run
#   ./scripts/run_gazebo_drop_zone_bench.sh gui:=false       # headless

# No -u (nounset): ROS2's own setup.bash files reference variables like
# AMENT_TRACE_SETUP_FILES without a default-value guard, assuming bash's
# normal "unset expands to empty" behavior -- nounset breaks sourcing them.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/scripts/activate_env.sh"

BENCH_WORLD="$(ros2 pkg prefix cuadc_sim)/share/cuadc_sim/worlds/cuadc_field_drop_zone_bench.sdf"

source ~/gazebo_ros2_ws/install/setup.bash
export GZ_VERSION=harmonic

exec ros2 launch cuadc_sim gazebo_bringup.launch.py "world:=$BENCH_WORLD" "$@"
