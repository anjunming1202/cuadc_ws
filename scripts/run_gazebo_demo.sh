#!/bin/bash
# Convenience wrapper for the full Gazebo simulation: sources the common
# env via activate_env.sh, adds the Gazebo-specific extras, then execs
# gazebo_bringup.launch.py.
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
source "$REPO_ROOT/scripts/activate_env.sh"

source ~/gazebo_ros2_ws/install/setup.bash
export GZ_VERSION=harmonic

exec ros2 launch cuadc_sim gazebo_bringup.launch.py "$@"
