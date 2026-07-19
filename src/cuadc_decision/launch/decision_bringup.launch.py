"""起决策节点。跟接口层、感知层之间只通过话题/service 通信（见
docs/spec/04-interfaces.md、docs/spec/05-perception-decision-interface.md），
这里不负责起那两层，组合闭环验证时单独把三个 launch 文件一起拉起来。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tick_rate_arg = DeclareLaunchArgument(
        'tick_rate_hz',
        default_value='10.0',
        description='状态机 tick 频率',
    )

    decision_node = Node(
        package='cuadc_decision',
        executable='decision_node',
        name='cuadc_decision',
        output='screen',
        parameters=[{'tick_rate_hz': LaunchConfiguration('tick_rate_hz')}],
    )

    return LaunchDescription([tick_rate_arg, decision_node])
