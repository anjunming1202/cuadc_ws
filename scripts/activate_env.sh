# Source this file, don't execute it: it exports environment variables
# into the *current* shell (ROS 2 + this workspace's install +
# AMENT_PREFIX_PATH workaround). Running it as ./scripts/activate_env.sh
# only affects a throwaway subshell and has no effect on your terminal.
#
# Workspace-wide base env, not scoped to any one package -- every package
# built in this workspace hits the same local colcon quirk (see
# docs/lessons.md 2026-07-09 entry), so this covers all of them generically
# instead of hardcoding one package name. Package-specific extras (like
# gazebo_bringup's ~/gazebo_ros2_ws + GZ_VERSION) still layer on top in
# their own script, e.g. run_gazebo_demo.sh.
#
# Usage: source scripts/activate_env.sh
#   then: ros2 launch cuadc_interface interface_bringup.launch.py
#         ./scripts/run_flight_check_sim.sh
#   Run once per terminal -- further ros2 run/launch calls in the same
#   shell don't need it again. A new terminal is a new shell process with
#   its own environment, so it needs its own source.

# No set -e/-u here: this gets sourced into an interactive shell, and
# those options would silently change that shell's behavior for the rest
# of the session, not just for this script.

_ACTIVATE_ENV_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$_ACTIVATE_ENV_REPO_ROOT"

# ROS distro differs by machine: dev workstation = Humble, 伴随机(树莓派) = Jazzy.
# Source whichever is installed (prefer an already-selected $ROS_DISTRO), so the
# same scripts run on both without editing. Add a distro here if a third appears.
if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  source "/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "activate_env.sh: no ROS found under /opt/ros (looked for \$ROS_DISTRO, jazzy, humble)" >&2
fi
source install/setup.bash
# Local colcon quirk workaround, see docs/lessons.md 2026-07-09 entry --
# loop over every built package instead of naming them one by one, so a
# newly added package doesn't need this script touched again.
for _pkg_install_dir in "$_ACTIVATE_ENV_REPO_ROOT"/install/*/; do
  export AMENT_PREFIX_PATH="${_pkg_install_dir%/}:$AMENT_PREFIX_PATH"
done
unset _pkg_install_dir

# Deliberately namespaced (not REPO_ROOT) -- this file is sourced (not
# executed) into the caller's shell, so a plain REPO_ROOT here would
# clobber a same-named variable the caller script still needs after
# sourcing us (e.g. run_gazebo_demo.sh declares its
# own REPO_ROOT before calling this).
unset _ACTIVATE_ENV_REPO_ROOT
echo "cuadc_ws env activated (ROS ${ROS_DISTRO:-?} + AMENT_PREFIX_PATH workaround)"
