"""Bring up ArduPilot SITL connected to Gazebo physics, plus a ROS2 bridge
for the onboard camera.

Gazebo (new-generation "gz sim") and the ardupilot_gazebo plugin are
external dependencies (not vendored in this repo, see
docs/spec/working-notes.md) -- both must already be installed/built;
this launch file only invokes them. ros_gz_bridge comes from the
separate ardupilot_gz workspace (~/gazebo_ros2_ws by default); its
install/setup.bash must be sourced before running this launch file, in
addition to this repo's own install/setup.bash.

This deliberately does NOT use ardupilot_gz_bringup's own launch files
(e.g. iris_runway.launch.py): those spawn the vehicle dynamically
through robot_state_publisher, which crashes in this environment due to
an unresolved upstream bug in ros/sdformat_urdf (model:// vs package://
URI resolution -- see docs/lessons.md). Instead this loads a world file
with the vehicle already statically included (the same approach already
verified end-to-end without ROS2 involved) and bridges only the camera
topic into ROS2 ourselves via ros_gz_bridge's parameter_bridge.

Runs SITL with MAVProxy enabled, same dual-output convention as
sitl_bringup.launch.py: udp:14550 free for QGroundControl, udp:14551
reserved for our own scripts/nodes.

First run in a fresh sitl_run_dir needs a EEPROM wipe, e.g.:
    ros2 launch cuadc_sim gazebo_bringup.launch.py extra_sitl_args:=-w
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ardupilot_path_arg = DeclareLaunchArgument(
        'ardupilot_path',
        default_value=os.path.expanduser('~/ardupilot'),
        description='Path to an ArduPilot source checkout providing Tools/autotest/sim_vehicle.py',
    )
    ardupilot_gazebo_path_arg = DeclareLaunchArgument(
        'ardupilot_gazebo_path',
        default_value=os.path.expanduser('~/ardupilot_gazebo'),
        description='Path to the built ardupilot_gazebo plugin (provides its worlds/models and plugin .so files)',
    )
    sitl_run_dir_arg = DeclareLaunchArgument(
        'sitl_run_dir',
        default_value=os.path.expanduser('~/ardupilot_run_gazebo_physics'),
        description='Scratch directory for SITL state/logs when using Gazebo physics',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='iris_runway.sdf',
        description=(
            'Either a filename under <ardupilot_gazebo_path>/worlds/ (e.g. the upstream '
            'iris_runway.sdf), or an absolute path to a world file elsewhere (e.g. one of '
            "our own under cuadc_sim/worlds/) -- vehicle must already be statically "
            'included in the world -- avoids the broken robot_state_publisher spawn path '
            '(see docs/lessons.md)'
        ),
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Show the Gazebo GUI window (set false for headless verification)',
    )
    extra_sitl_args_arg = DeclareLaunchArgument(
        'extra_sitl_args',
        default_value='',
        description="Extra args appended to sim_vehicle.py, e.g. '-w' to wipe EEPROM on first run",
    )
    camera_gz_topic_arg = DeclareLaunchArgument(
        'camera_gz_topic',
        default_value=(
            '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link'
            '/sensor/camera/image'
        ),
        description=(
            "Gazebo camera image topic to bridge into ROS2 -- specific to the "
            "iris_with_gimbal model in iris_runway.sdf; override if 'world' is changed"
        ),
    )
    # Exported inline on gz sim's own command line (rather than via
    # SetEnvironmentVariable) so this specific process always uses
    # ardupilot_gazebo_path's assets, regardless of what a parent shell
    # may have already exported (e.g. after sourcing the separate
    # ardupilot_gz workspace's install/setup.bash for ros_gz_bridge --
    # that workspace vendors its own, differently-behaved copy of
    # ardupilot_gazebo on the 'ros2' branch). SDF_PATH must be
    # overridden too, not just GZ_SIM_RESOURCE_PATH: libsdformat
    # resolves model:// URIs via SDF_PATH itself before gz-sim's own
    # GZ_SIM_RESOURCE_PATH-based callback gets a chance, so leaving an
    # inherited SDF_PATH pointed at the other copy silently picks the
    # wrong (broken) model; see docs/lessons.md.
    gz_env_prefix = [
        'GZ_SIM_SYSTEM_PLUGIN_PATH=',
        PathJoinSubstitution([LaunchConfiguration('ardupilot_gazebo_path'), 'build']),
        ' GZ_SIM_RESOURCE_PATH=',
        PathJoinSubstitution([LaunchConfiguration('ardupilot_gazebo_path'), 'models']),
        ':',
        PathJoinSubstitution([LaunchConfiguration('ardupilot_gazebo_path'), 'worlds']),
        ' SDF_PATH=',
        PathJoinSubstitution([LaunchConfiguration('ardupilot_gazebo_path'), 'worlds']),
        ':',
        PathJoinSubstitution([LaunchConfiguration('ardupilot_gazebo_path'), 'models']),
        ' ',
    ]

    # 'world' may be a bare filename (resolved under ardupilot_gazebo_path's
    # worlds/ dir, the upstream-example case) or an absolute path (our own
    # worlds under cuadc_sim/worlds/, which don't live there).
    world_path = PythonExpression([
        "'", LaunchConfiguration('world'), "' if '", LaunchConfiguration('world'),
        "'.startswith('/') else '",
        PathJoinSubstitution(
            [LaunchConfiguration('ardupilot_gazebo_path'), 'worlds', LaunchConfiguration('world')]),
        "'",
    ])

    gz_sim_gui = ExecuteProcess(
        cmd=[gz_env_prefix + ['gz sim -v4 -r ', world_path]],
        shell=True,
        name='gz_sim',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )
    gz_sim_headless = ExecuteProcess(
        cmd=[gz_env_prefix + ['gz sim -v4 -s -r ', world_path]],
        shell=True,
        name='gz_sim',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    sitl_process = ExecuteProcess(
        cmd=[[
            LaunchConfiguration('ardupilot_path'), '/Tools/autotest/sim_vehicle.py',
            ' -v ArduCopter -f gazebo-iris --model JSON',
            # sim_vehicle.py's -f <frame> does NOT actually apply the
            # frame's default_params_filename mapping from
            # vehicleinfo.json for the main vehicle binary (that
            # mapping is only read by the unrelated CAN-peripheral code
            # path) -- without this, FRAME_CLASS stays 0/undefined and
            # QGC refuses to arm ("PreArm: Motors: Check frame class
            # and type"). --add-param-file is the documented way to
            # layer a parameter file on top of the embedded defaults;
            # see docs/lessons.md.
            ' --add-param-file ', LaunchConfiguration('ardupilot_path'),
            '/Tools/autotest/default_params/gazebo-iris.parm',
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

    camera_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[
            [LaunchConfiguration('camera_gz_topic'), '@sensor_msgs/msg/Image[gz.msgs.Image'],
        ],
        remappings=[
            (LaunchConfiguration('camera_gz_topic'), '/camera/image_raw'),
        ],
    )

    return LaunchDescription([
        ardupilot_path_arg,
        ardupilot_gazebo_path_arg,
        sitl_run_dir_arg,
        world_arg,
        gui_arg,
        extra_sitl_args_arg,
        camera_gz_topic_arg,
        gz_sim_gui,
        gz_sim_headless,
        sitl_process,
        camera_bridge_node,
    ])
