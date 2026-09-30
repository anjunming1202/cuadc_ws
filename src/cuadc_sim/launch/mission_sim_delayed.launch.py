"""压测台专用（sim-only）：mission_bringup 原样 + 相机→CV 图像延迟。
image_delay_node 延迟转发 Image（保留拍照戳），SetRemap 让 group 内的感知改订延迟后
话题；生产链不含这套。量值取 cv-field-test profiling 实测档，见
docs/spec/sim-stress-testbed.md §3.2/§3.3。

mission_bringup.launch.py 在仓库根 launch/（不属于任何包），路径由调用方传入
（见 run_mission_sim_delayed.sh）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

_RAW = '/camera/image_raw'
_DELAYED = '/camera/image_delayed'


def generate_launch_description():
    mission_path_arg = DeclareLaunchArgument(
        'mission_bringup_path', description='仓库根 launch/mission_bringup.launch.py 的绝对路径')
    connection_arg = DeclareLaunchArgument('connection', default_value='udp:127.0.0.1:14550')
    scenario_arg = DeclareLaunchArgument('scenario', default_value='full_mission')
    delay_arg = DeclareLaunchArgument(
        'delay_ms', default_value='0.0',
        description='相机→CV 图像延迟（毫秒，保留拍照戳）；0=直通')

    delay_node = Node(
        package='cuadc_sim', executable='image_delay_node', name='image_delay',
        output='screen',
        parameters=[{'delay_ms': LaunchConfiguration('delay_ms'), 'use_sim_time': True}],
        remappings=[('image_in', _RAW), ('image_out', _DELAYED)])

    # SetRemap 只作用于本 group 内的 mission 节点（订 _RAW 的只有感知）；
    # 延迟节点在 group 外、用自己的显式 remap，不受影响。
    mission = GroupAction([
        SetRemap(_RAW, _DELAYED),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(LaunchConfiguration('mission_bringup_path')),
            launch_arguments={
                'connection': LaunchConfiguration('connection'),
                'profile': 'sim',
                'scenario': LaunchConfiguration('scenario'),
                'frame_source': 'ros',
            }.items()),
    ])

    return LaunchDescription([mission_path_arg, connection_arg, scenario_arg, delay_arg,
                              delay_node, mission])
