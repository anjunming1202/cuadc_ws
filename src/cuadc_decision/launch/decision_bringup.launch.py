"""起决策节点。跟接口层、感知层之间只通过话题/service 通信（见
docs/spec/04-interfaces.md、docs/spec/05-perception-decision-interface.md），
这里不负责起那两层，组合闭环验证时单独把三个 launch 文件一起拉起来。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    tick_rate_arg = DeclareLaunchArgument(
        'tick_rate_hz',
        default_value='10.0',
        description='状态机 tick 频率',
    )
    initial_state_arg = DeclareLaunchArgument(
        'initial_state',
        default_value='INIT',
        description='硬件单点测试用：跳过前面几个状态直接从某个状态起步，'
                     '比如 NAVIGATE_TO_DROP_ZONE。默认 INIT 跟正常任务一样',
    )
    stop_after_drop_arg = DeclareLaunchArgument(
        'stop_after_drop',
        default_value='false',
        description='硬件单点测试用：两次投放完成后直接 RTL，不去侦察区',
    )
    profile_arg = DeclareLaunchArgument(
        'profile', default_value='sim',
        description='接近算法 sim/real config 档，选 approaches/<approach>.<profile>.yaml，仿真 sim、真机 real',
    )

    decision_node = Node(
        package='cuadc_decision',
        executable='decision_node',
        name='cuadc_decision',
        output='screen',
        parameters=[{
            'tick_rate_hz': LaunchConfiguration('tick_rate_hz'),
            'initial_state': LaunchConfiguration('initial_state'),
            # launch 参数本质是字符串，'true'/'false' 要显式转成 bool，
            # 不然节点里 declare_parameter(..., False) 会因为类型不匹配报错
            'stop_after_drop': ParameterValue(LaunchConfiguration('stop_after_drop'), value_type=bool),
            'profile': LaunchConfiguration('profile'),
        }],
    )

    return LaunchDescription([tick_rate_arg, initial_state_arg, stop_after_drop_arg, profile_arg, decision_node])
