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
    profile_arg = DeclareLaunchArgument(
        'profile', default_value='sim',
        description='环境档 sim/real，选各算法/配置层的 .sim/.real 文件，仿真 sim、真机 real',
    )
    scenario_arg = DeclareLaunchArgument(
        'scenario', default_value='full_mission',
        description='跑哪种场景，选 mission/scenarios/<scenario>.yaml（含 initial_state 与跑哪些阶段）：'
                     'full_mission / drop_zone_bench / recon_zone_bench，见 docs/spec/mission-config-layer.md',
    )

    approach_arg = DeclareLaunchArgument(
        'approach', default_value='',
        description='接近模块名（cone_descent_ibvs / position_lock_descent / ...），空 = approaches.ACTIVE')
    approach_variant_arg = DeclareLaunchArgument(
        'approach_variant', default_value='',
        description='叠加 <approach>.<variant>.yaml（如 lowgain；逗号分隔可多个），空 = 不叠加')

    survey_arg = DeclareLaunchArgument(
        'survey', default_value='',
        description='侦察覆盖模块名（coverage_scan / detect_count），空 = surveys.ACTIVE')

    first_drop_port_arg = DeclareLaunchArgument(
        'first_drop_port', default_value='0', description='第一投用的投放口：0 左 / 1 右，之后轮换')

    decision_node = Node(
        package='cuadc_decision',
        executable='decision_node',
        name='cuadc_decision',
        output='screen',
        parameters=[{
            'tick_rate_hz': LaunchConfiguration('tick_rate_hz'),
            'profile': LaunchConfiguration('profile'),
            'scenario': LaunchConfiguration('scenario'),
            'approach': LaunchConfiguration('approach'),
            'approach_variant': LaunchConfiguration('approach_variant'),
            'survey': LaunchConfiguration('survey'),
            'first_drop_port': LaunchConfiguration('first_drop_port'),
        }],
    )

    return LaunchDescription([tick_rate_arg, profile_arg, scenario_arg, approach_arg, approach_variant_arg,
                              survey_arg, first_drop_port_arg, decision_node])
