"""Bring up ArduPilot SITL (ArduCopter) with built-in physics (no Gazebo).

ArduPilot SITL is an external dependency (not vendored in this repo, see
docs/spec/working-notes.md) -- it must already be cloned and its Python
requirements installed; this launch file only invokes it.

Lightweight sandbox for testing MAVLink command/telemetry logic (the
interface layer) without the overhead of Gazebo -- see
src/cuadc_sim/README.md for how this differs from gazebo_bringup.launch.py.
This launch file only stands up the simulated world; it deliberately
does not run any task/verification node -- run those separately, e.g.:
    ros2 run cuadc_sim sitl_smoke_test

Runs with MAVProxy enabled so more than one client can be connected at
once: MAVProxy's default GCS output (udp:127.0.0.1:14550) is free for
QGroundControl or another GCS, and a second output (udp:127.0.0.1:14551)
is reserved for our own scripts/nodes.

First run in a fresh sitl_run_dir needs a EEPROM wipe, e.g.:
    ros2 launch cuadc_sim sitl_bringup.launch.py extra_sitl_args:=-w
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression


def generate_launch_description():
    ardupilot_path_arg = DeclareLaunchArgument(
        'ardupilot_path',
        default_value=os.path.expanduser('~/ardupilot'),
        description='Path to an ArduPilot source checkout providing Tools/autotest/sim_vehicle.py',
    )
    sitl_run_dir_arg = DeclareLaunchArgument(
        'sitl_run_dir',
        default_value=os.path.expanduser('~/ardupilot_run_builtin_physics'),
        description='Scratch directory for SITL state/logs (kept out of the repo and out of the ardupilot checkout)',
    )
    frame_arg = DeclareLaunchArgument(
        'frame',
        default_value='quad',
        description='ArduCopter frame type passed to sim_vehicle.py -f',
    )
    extra_sitl_args_arg = DeclareLaunchArgument(
        'extra_sitl_args',
        default_value='',
        description="Extra args appended to sim_vehicle.py, e.g. '-w' to wipe EEPROM on first run",
    )
    imperfections_arg = DeclareLaunchArgument(
        'imperfections', default_value='true',
        description='Wind / sensor noise on by default (same as gazebo_bringup); false = clean sim')
    sim_realism_params_arg = DeclareLaunchArgument(
        'sim_realism_params',
        default_value=os.path.join(
            get_package_share_directory('cuadc_sim'), 'config', 'sim_realism.parm'),
        description="SITL .parm layered last; '' = none. Ignored when imperfections:=false.")

    sitl_process = ExecuteProcess(
        cmd=[[
            LaunchConfiguration('ardupilot_path'), '/Tools/autotest/sim_vehicle.py',
            ' -v ArduCopter -f ', LaunchConfiguration('frame'),
            # Our own param overrides (e.g. RTL_ALT_M), layered on top of
            # whatever frame defaults sim_vehicle.py loads -- kept in sync
            # with gazebo_bringup.launch.py's identical override so RTL
            # behavior doesn't depend on which sandbox is used.
            ' --add-param-file ',
            PathJoinSubstitution([get_package_share_directory('cuadc_sim'),
                                   'config', 'mission_defaults.parm']),
            PythonExpression([
                "' --add-param-file ' + '", LaunchConfiguration('sim_realism_params'),
                "' if '", LaunchConfiguration('imperfections'), "'.lower() == 'true' and '",
                LaunchConfiguration('sim_realism_params'), "' else ''"]),
            # SITL's default build has HAL_WITH_ESC_TELEM=0 (no CAN/serial
            # RCOut), which compiles out the entire body of
            # Copter::takeoff_check() -- the only place that ever clears
            # the motors' spool-up block. Without this, motors latch in
            # GROUND_IDLE forever after arming and the vehicle auto-disarms
            # instead of climbing. TKOFF_RPM_MIN defaults to 0 so forcing
            # this on does not require real ESC RPM telemetry. See
            # src/cuadc_sim/README.md "已解决问题".
            ' --configure-define=HAL_WITH_ESC_TELEM=1',
            ' --no-wsl2-network --use-dir ', LaunchConfiguration('sitl_run_dir'),
            ' -m "--out 127.0.0.1:14551"',
            ' ', LaunchConfiguration('extra_sitl_args'),
        ]],
        shell=True,
        cwd=LaunchConfiguration('ardupilot_path'),
        name='ardupilot_sitl',
        output='screen',
    )

    return LaunchDescription([
        ardupilot_path_arg,
        sitl_run_dir_arg,
        frame_arg,
        extra_sitl_args_arg,
        imperfections_arg,
        sim_realism_params_arg,
        sitl_process,
    ])
