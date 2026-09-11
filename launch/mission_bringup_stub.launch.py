"""桩感知任务节点总拉起：接口层 + 桩感知（地面真值）+ 决策。**不是**正式
任务用的文件——桩感知从场地已知坐标直接发布"检测结果"，不跑真实 CV
算法，只用于快速迭代决策层逻辑（不想每次改状态机都要等真实检测收敛）。
正式任务/完整仿真验证一律用 mission_bringup.launch.py（真实 CV 感知），
不要在那份文件上加参数切回桩感知——两份文件故意保持独立，文件名本身
就说明用途，不用记哪个参数默认值是什么，见 mission_bringup.launch.py
文件头的说明。

同样不包含仿真世界本身、不包含任务开始触发，见 mission_bringup.launch.py
对应段落。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    connection_arg = DeclareLaunchArgument(
        'connection',
        default_value='udp:127.0.0.1:14550',
        description='接口层 MAVLink 连接目标，仿真默认连本机 SITL',
    )

    interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('cuadc_interface'), 'launch', 'interface_bringup.launch.py'])),
        launch_arguments={'connection': LaunchConfiguration('connection')}.items(),
    )

    stub_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('cuadc_perception'), 'launch', 'stub_perception_bringup.launch.py'])),
    )

    decision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('cuadc_decision'), 'launch', 'decision_bringup.launch.py'])),
    )

    return LaunchDescription([connection_arg, interface_launch, stub_perception_launch, decision_launch])
