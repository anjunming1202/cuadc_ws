"""正式任务节点总拉起：接口层 + 真实 CV 感知 + 决策，仿真和真机都用这一份
（真机部署时启动这三个节点的方式不变，只是 connection 参数从 SITL 的 UDP
换成真实串口）。不包含仿真世界本身（Gazebo/SITL）——那是另一层，见
scripts/run_gazebo_demo.sh；也不包含任务开始触发（模拟/真机 GPIO 开关），
按设计要单独一个终端触发，见 README.md Quickstart。

放在仓库根目录而不是某个包的 launch/ 下：这份文件组合了三个包的节点，
不属于其中任何一个包自己的职责（比如放进 cuadc_decision 会显得 decision
"拥有"了 interface/perception，但 decision 的 package.xml 并不依赖它们）。
`ros2 launch` 支持直接给 .launch.py 路径运行、不需要挂在某个包下。

这份文件永远起真实 CV 感知（cv_perception_node），不提供切换到桩感知的
参数——桩感知（地面真值，快速迭代决策层用）是另一份完全独立的文件
mission_bringup_stub.launch.py，两者故意不共用一个"模式开关"：之前用
`perception_mode` 参数二选一、默认 stub，结果地面站调试时"起了任务节点"
实际起的是桩感知，debug 图像/CV profiling 全是空的，排查了很久才发现是
默认值的问题。一份文件只做一件事，用哪个由文件名直接决定，不需要记参数
默认值是什么。connection 不受这条限制——它不改变"起的是哪个感知"，只是
接口层连去哪，仿真/真机两边各自的启动脚本（见 scripts/run_mission_sim.sh、
scripts/run_mission_hardware.sh）各自把它写死，调这份文件时不需要再传。
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
        description='接口层 MAVLink 连接目标，仿真默认连本机 SITL，真机部署时改成串口',
    )
    # profile 跟 connection 同一个道理：sim/真机各自的启动脚本把它写死
    # （run_mission_sim.sh=sim / run_mission_hardware.sh=real），选感知检测器
    # 的 sim/real config 档，见 cuadc_perception/README.md 配置分层。
    profile_arg = DeclareLaunchArgument(
        'profile', default_value='sim',
        description='感知检测器 sim/real config 档，仿真 sim、真机 real',
    )
    # scenario 只到决策层（选 mission/scenarios/<scenario>.yaml：initial_state + 跑哪些阶段 +
    # SEARCH 覆盖）；感知/接口层不认场景。各启动脚本把它写死（正式任务 full_mission、投放区
    # 单练 drop_zone_bench…），见 docs/spec/mission-config-layer.md。
    scenario_arg = DeclareLaunchArgument(
        'scenario', default_value='full_mission',
        description='任务场景，选 mission/scenarios/<scenario>.yaml，正式任务 full_mission',
    )
    # 真实 CV 接线的透传参数：只转给 cv_perception，接口/决策层不认。默认 sim 友好
    # （frame_source=ros、预览关、sim 内参），仿真脚本一个都不用传、老路不变；真机
    # 脚本（run_mission_hardware.sh）source real_camera_intrinsics.sh 后成套传进来
    # （见 launch/cv_perception_bringup.launch.py 里这些参数的完整说明）。
    cv_passthrough_args = [
        DeclareLaunchArgument('frame_source', default_value='ros', description='取帧源：sim=ros / real=shm'),
        DeclareLaunchArgument('preview_enabled', default_value='false', description='真机标注预览推本地 preview_port'),
        DeclareLaunchArgument('fx', default_value='205.5', description='相机水平焦距（像素），真机传标定值'),
        DeclareLaunchArgument('fy', default_value='205.5', description='相机垂直焦距（像素）'),
        DeclareLaunchArgument('cx', default_value='320.0', description='主点 x（像素）'),
        DeclareLaunchArgument('cy', default_value='240.0', description='主点 y（像素）'),
        DeclareLaunchArgument('preview_port', default_value='5604', description='真机预览本地推流端口'),
    ]

    interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('cuadc_interface'), 'launch', 'interface_bringup.launch.py'])),
        launch_arguments={'connection': LaunchConfiguration('connection')}.items(),
    )

    cv_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('cuadc_perception'), 'launch', 'cv_perception_bringup.launch.py'])),
        launch_arguments={
            'profile': LaunchConfiguration('profile'),
            'frame_source': LaunchConfiguration('frame_source'),
            'preview_enabled': LaunchConfiguration('preview_enabled'),
            'fx': LaunchConfiguration('fx'),
            'fy': LaunchConfiguration('fy'),
            'cx': LaunchConfiguration('cx'),
            'cy': LaunchConfiguration('cy'),
            'preview_port': LaunchConfiguration('preview_port'),
        }.items(),
    )

    decision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('cuadc_decision'), 'launch', 'decision_bringup.launch.py'])),
        launch_arguments={
            'profile': LaunchConfiguration('profile'),
            'scenario': LaunchConfiguration('scenario'),
        }.items(),
    )

    return LaunchDescription([connection_arg, profile_arg, scenario_arg,
                              *cv_passthrough_args,
                              interface_launch, cv_perception_launch, decision_launch])
