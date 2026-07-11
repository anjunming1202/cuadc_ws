# Source this file, don't execute it: it exports environment variables
# into the *current* shell (ROS Humble + this workspace's install +
# AMENT_PREFIX_PATH workaround). Running it as ./scripts/activate_env.sh
# only affects a throwaway subshell and has no effect on your terminal.
#
# This is the intersection of what every cuadc_sim entry point needs --
# sitl_bringup, gazebo_bringup, and the MAVLink-only test nodes
# (sitl_smoke_test / flight_acceptance_test) all need exactly this much.
# gazebo_bringup additionally needs ~/gazebo_ros2_ws + GZ_VERSION, added
# on top of this in run_gazebo_demo.sh -- the test nodes deliberately
# don't need those, since they only talk MAVLink and are backend-agnostic.
#
# Usage: source scripts/activate_env.sh
#   then: ros2 run cuadc_sim sitl_smoke_test
#         ros2 run cuadc_sim flight_acceptance_test
#   Run once per terminal -- further ros2 run/launch calls in the same
#   shell don't need it again. A new terminal is a new shell process with
#   its own environment, so it needs its own source.

# No set -e/-u here: this gets sourced into an interactive shell, and
# those options would silently change that shell's behavior for the rest
# of the session, not just for this script.

CUADC_SIM_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$CUADC_SIM_REPO_ROOT"

source /opt/ros/humble/setup.bash
source install/setup.bash
# Local colcon quirk workaround, see docs/lessons.md 2026-07-09 entry.
export AMENT_PREFIX_PATH="$CUADC_SIM_REPO_ROOT/install/cuadc_sim:$AMENT_PREFIX_PATH"

unset CUADC_SIM_REPO_ROOT
echo "cuadc_sim env activated (ROS Humble + AMENT_PREFIX_PATH workaround)"
