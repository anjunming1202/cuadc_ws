#!/bin/bash
# Convenience wrapper for the lightweight SITL sandbox (no Gazebo): sources
# the workspace + env var sitl_bringup.launch.py needs, then execs it.
# See src/cuadc_sim/README.md for what this actually launches.
#
# Usage: ./scripts/run_sitl_demo.sh [extra ros2 launch args]
#   ./scripts/run_sitl_demo.sh                       # normal run
#   ./scripts/run_sitl_demo.sh extra_sitl_args:=-w   # first run in a fresh sitl_run_dir

# No -u (nounset): ROS2's own setup.bash files reference variables like
# AMENT_TRACE_SETUP_FILES without a default-value guard, assuming bash's
# normal "unset expands to empty" behavior -- nounset breaks sourcing them.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

source /opt/ros/humble/setup.bash
source install/setup.bash
# Local colcon quirk workaround, see docs/lessons.md 2026-07-09 entry.
export AMENT_PREFIX_PATH="$REPO_ROOT/install/cuadc_sim:$AMENT_PREFIX_PATH"

exec ros2 launch cuadc_sim sitl_bringup.launch.py "$@"
