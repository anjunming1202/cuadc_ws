"""Bring up ArduPilot SITL (ArduCopter) and run the closed-loop smoke test.

ArduPilot SITL is an external dependency (not vendored in this repo, see
docs/spec/working-notes.md) -- it must already be cloned and its Python
requirements installed; this launch file only invokes it.

Runs with MAVProxy enabled so more than one client can be connected at
once: MAVProxy's default GCS output (udp:127.0.0.1:14550) is free for
QGroundControl or another GCS, and a second output (udp:127.0.0.1:14551)
is reserved for our own smoke-test node.

First run in a fresh sitl_run_dir needs a EEPROM wipe, e.g.:
    ros2 launch cuadc_sim sitl_bringup.launch.py extra_sitl_args:=-w
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    connection_arg = DeclareLaunchArgument(
        'connection',
        default_value='udp:127.0.0.1:14551',
        description=(
            'MAVLink connection string the smoke-test node should connect to. '
            'MAVProxy reserves udp:14550 for a GCS (e.g. QGroundControl); '
            "14551 is this node's own dedicated output."
        ),
    )
    extra_sitl_args_arg = DeclareLaunchArgument(
        'extra_sitl_args',
        default_value='',
        description="Extra args appended to sim_vehicle.py, e.g. '-w' to wipe EEPROM on first run",
    )
    run_guided_check_arg = DeclareLaunchArgument(
        'run_guided_check',
        default_value='true',
        description='Also arm, takeoff, and send one GUIDED position target',
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

    smoke_test_node = Node(
        package='cuadc_sim',
        executable='sitl_smoke_test',
        name='sitl_smoke_test',
        output='screen',
        parameters=[{
            'connection': LaunchConfiguration('connection'),
            'run_guided_check': ParameterValue(
                LaunchConfiguration('run_guided_check'), value_type=bool),
        }],
    )

    return LaunchDescription([
        ardupilot_path_arg,
        sitl_run_dir_arg,
        frame_arg,
        connection_arg,
        extra_sitl_args_arg,
        run_guided_check_arg,
        sitl_process,
        smoke_test_node,
    ])
