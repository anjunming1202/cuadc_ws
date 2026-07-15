"""起接口层节点。仿真连 SITL、真机连真飞控只是 connection 这一个参数
不同（见 docs/spec/00-index.md"仿真 vs 真机不是两套代码"），本身不含
任务/规划逻辑。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    connection_arg = DeclareLaunchArgument(
        'connection',
        default_value='udp:127.0.0.1:14550',
        description='MAVLink 连接目标，仿真默认连本机 SITL，真机部署时改成串口',
    )

    interface_node = Node(
        package='cuadc_interface',
        executable='interface_node',
        name='cuadc_interface',
        output='screen',
        parameters=[{'connection': LaunchConfiguration('connection')}],
    )

    return LaunchDescription([connection_arg, interface_node])
