"""起桩感知节点。真实感知节点做好之后会是一个平行的 launch 文件，两者
发布同一份契约（见 docs/spec/05-perception-decision-interface.md），
决策层不需要区分对面是哪一个。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='10.0',
        description='检测结果发布频率',
    )
    position_noise_arg = DeclareLaunchArgument(
        'position_noise_stddev',
        default_value='0.0',
        description='位置高斯噪声标准差（米），0 表示直接发地面真值，用于测试决策层对噪声的鲁棒性',
    )
    visibility_radius_arg = DeclareLaunchArgument(
        'visibility_radius_m',
        default_value='2.5',
        description='只报告距离当前水平位置这个半径以内的圆筒，模拟相机看不到全场'
                     '（2.5m 是刻意选小的：4.0m 的话侦察区单个悬停点就能看全 5 个筒，'
                     '测不出 SURVEY_RECON_ZONE 的多航点扫描逻辑，见 mission_logic.py 里'
                     ' RECON_SCAN_WAYPOINTS 的覆盖半径计算）',
    )

    stub_perception_node = Node(
        package='cuadc_perception',
        executable='stub_perception_node',
        name='cuadc_perception_stub',
        output='screen',
        parameters=[{
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            'position_noise_stddev': LaunchConfiguration('position_noise_stddev'),
            'visibility_radius_m': LaunchConfiguration('visibility_radius_m'),
        }],
    )

    return LaunchDescription([
        publish_rate_arg, position_noise_arg, visibility_radius_arg, stub_perception_node,
    ])
