"""压测台专用（sim-only）：跟 launch/mission_bringup.launch.py 起同样的三个任务
节点（接口/真实 CV 感知/决策），唯一区别是在感知↔决策之间插一个
detection_delay_node，把检测延迟 delay_ms 再交给决策，用来在仿真里建模真实检测
流水线时延（见 docs/spec/sim-stress-testbed.md §3.2）。

时延注入完全不碰任何生产文件：靠 SetRemap 把感知的输出话题 /perception/detections
改道到 /perception/detections_predelay——SetRemap 只作用于**本 group 内**的节点，
也就是被包含的 cv_perception_bringup（决策不在这个 group 里，它对
/perception/detections 的订阅不受影响）；延迟节点再把 _predelay 延迟后发回
/perception/detections 给决策。生产链 mission_bringup 里没有这个 launch、没有延迟
节点、没有 remap，真机/正式仿真感知→决策零延迟直连。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

_DETECTIONS = '/perception/detections'          # 感知↔决策的契约话题
_PREDELAY = '/perception/detections_predelay'   # 感知先发这里，延迟节点再转回 _DETECTIONS


def generate_launch_description():
    connection_arg = DeclareLaunchArgument(
        'connection', default_value='udp:127.0.0.1:14550',
        description='接口层 MAVLink 连接目标，压测台连本机 SITL')
    profile_arg = DeclareLaunchArgument(
        'profile', default_value='sim', description='感知检测器 sim/real 档，压测台用 sim')
    delay_arg = DeclareLaunchArgument(
        'delay_ms', default_value='0.0',
        description='检测发布延迟（毫秒），建模相机→CV→决策实测时延；0=直通。见 docs/spec/sim-stress-testbed.md')

    interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('cuadc_interface'), 'launch', 'interface_bringup.launch.py'])),
        launch_arguments={'connection': LaunchConfiguration('connection')}.items())

    # 感知：SetRemap 把它的 DetectionArray 输出改道到 _predelay。只作用于本 group
    # 内被包含的 cv_perception_bringup，感知节点本身不知情、也没改。
    perception = GroupAction([
        SetRemap(_DETECTIONS, _PREDELAY),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('cuadc_perception'), 'launch', 'cv_perception_bringup.launch.py'])),
            launch_arguments={'profile': LaunchConfiguration('profile')}.items()),
    ])

    delay_node = Node(
        package='cuadc_sim', executable='detection_delay_node', name='detection_delay',
        output='screen',
        parameters=[{'delay_ms': LaunchConfiguration('delay_ms')}],
        remappings=[('detections_in', _PREDELAY), ('detections_out', _DETECTIONS)])

    decision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('cuadc_decision'), 'launch', 'decision_bringup.launch.py'])),
        launch_arguments={'profile': LaunchConfiguration('profile')}.items())

    return LaunchDescription([connection_arg, profile_arg, delay_arg,
                              interface, perception, delay_node, decision])
