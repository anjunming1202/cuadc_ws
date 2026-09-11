"""任务节点总拉起：接口层 + 感知 + 决策，仿真和真机都用这一份（真机部署
时启动这三个节点的方式不变，只是 connection 参数从 SITL 的 UDP 换成真实
串口）。不包含仿真世界本身（Gazebo/SITL）——那是另一层，见
scripts/run_gazebo_demo.sh；也不包含任务开始触发（模拟/真机 GPIO 开关），
按设计要单独一个终端触发，见 README.md Quickstart。

放在仓库根目录而不是某个包的 launch/ 下：这份文件组合了三个包的节点，
不属于其中任何一个包自己的职责（比如放进 cuadc_decision 会显得 decision
"拥有"了 interface/perception，但 decision 的 package.xml 并不依赖它们）。
`ros2 launch` 支持直接给 .launch.py 路径运行、不需要挂在某个包下，见
scripts/run_task_nodes.sh 的调用方式。

感知层有桩（cuadc_perception/stub_perception_node）和真实 CV
（cv_perception_node）两个实现，`perception_mode` 参数二选一，两者发布
同一份检测契约（docs/spec/05-perception-decision-interface.md），decision
不需要改。默认 stub，跟现有闭环验证行为保持一致，不受影响。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    connection_arg = DeclareLaunchArgument(
        'connection',
        default_value='udp:127.0.0.1:14550',
        description='接口层 MAVLink 连接目标，仿真默认连本机 SITL，真机部署时改成串口',
    )
    perception_mode_arg = DeclareLaunchArgument(
        'perception_mode',
        default_value='stub',
        description="'stub'（地面真值，快速迭代决策层用）或 'cv'（真实检测算法）",
    )
    is_stub = PythonExpression(["'", LaunchConfiguration('perception_mode'), "' == 'stub'"])

    interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('cuadc_interface'), 'launch', 'interface_bringup.launch.py'])),
        launch_arguments={'connection': LaunchConfiguration('connection')}.items(),
    )

    stub_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('cuadc_perception'), 'launch', 'stub_perception_bringup.launch.py'])),
        condition=IfCondition(is_stub),
    )

    cv_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('cuadc_perception'), 'launch', 'cv_perception_bringup.launch.py'])),
        condition=UnlessCondition(is_stub),
    )

    decision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('cuadc_decision'), 'launch', 'decision_bringup.launch.py'])),
    )

    return LaunchDescription([
        connection_arg, perception_mode_arg,
        interface_launch, stub_perception_launch, cv_perception_launch, decision_launch,
    ])
