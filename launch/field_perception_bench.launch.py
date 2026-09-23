"""感知单点测试（L1）拉起：接口层 + 真实 CV 感知，**不含决策、不发任何飞行指令**。
用于真机验证「取帧(shm)→检测→反投影→尺寸判别」这条链路本身——看 DetectionArray /
地面 CV 标注框对不对，飞机全程停在地面、飞手 RC 随时可接管。

为什么要接口层：感知反投影要当前位姿（接口层 republish 的遥测），没有接口层
DetectionArray 里的世界坐标就是空的。接口层只连底座 MAVLink 端点读遥测，本 launch
不起决策、不调任何会驱动飞行/投放的服务，因此不会有自主动作。

跟 launch/mission_bringup.launch.py 平级但去掉 decision 那一层；CV 透传参数含义见
launch/cv_perception_bringup.launch.py。真机由 scripts/run_field_l1_perception.sh 起
（source real_camera_intrinsics.sh 注入真实内参 + frame_source=shm + 预览）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    connection_arg = DeclareLaunchArgument(
        'connection', default_value='udp:127.0.0.1:14550',
        description='接口层 MAVLink 连接目标，真机连底座 mav_fanout 端点',
    )
    profile_arg = DeclareLaunchArgument(
        'profile', default_value='real',
        description='感知检测器 sim/real 档，感知单点测试默认 real',
    )
    cv_passthrough_args = [
        DeclareLaunchArgument('frame_source', default_value='shm'),
        DeclareLaunchArgument('preview_enabled', default_value='true'),
        DeclareLaunchArgument('fx', default_value='205.5'),
        DeclareLaunchArgument('fy', default_value='205.5'),
        DeclareLaunchArgument('cx', default_value='320.0'),
        DeclareLaunchArgument('cy', default_value='240.0'),
        DeclareLaunchArgument('preview_port', default_value='5604'),
        DeclareLaunchArgument('detection_log_enabled', default_value='false'),
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
            'detection_log_enabled': LaunchConfiguration('detection_log_enabled'),
        }.items(),
    )

    return LaunchDescription([connection_arg, profile_arg, *cv_passthrough_args,
                              interface_launch, cv_perception_launch])
