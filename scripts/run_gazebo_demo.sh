#!/bin/bash
# Convenience wrapper for the full Gazebo simulation: sources the two
# workspaces + env vars gazebo_bringup.launch.py needs, then execs it.
# See src/cuadc_sim/README.md for what this actually launches.
#
# Usage: ./scripts/run_gazebo_demo.sh [extra ros2 launch args]
#   ./scripts/run_gazebo_demo.sh                       # normal run
#   ./scripts/run_gazebo_demo.sh extra_sitl_args:=-w   # first run in a fresh sitl_run_dir
#   ./scripts/run_gazebo_demo.sh gui:=false             # headless

# No -u (nounset): ROS2's own setup.bash files reference variables like
# AMENT_TRACE_SETUP_FILES without a default-value guard, assuming bash's
# normal "unset expands to empty" behavior -- nounset breaks sourcing them.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/gazebo_ros2_ws/install/setup.bash
export GZ_VERSION=harmonic
# Local colcon quirk workaround, see docs/lessons.md 2026-07-09 entry.
export AMENT_PREFIX_PATH="$REPO_ROOT/install/cuadc_sim:$AMENT_PREFIX_PATH"

exec ros2 launch cuadc_sim gazebo_bringup.launch.py "$@"
