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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


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

    sitl_process = ExecuteProcess(
        cmd=[[
            LaunchConfiguration('ardupilot_path'), '/Tools/autotest/sim_vehicle.py',
            ' -v ArduCopter -f ', LaunchConfiguration('frame'),
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
        sitl_process,
    ])
