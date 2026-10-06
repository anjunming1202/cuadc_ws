"""起接口层节点。仿真连 SITL、真机连真飞控只是 connection 这一个参数
不同（见 docs/spec/00-index.md"仿真 vs 真机不是两套代码"），本身不含
任务/规划逻辑。"""

import cuadc_physical
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    connection_arg = DeclareLaunchArgument(
        'connection',
        default_value='udp:127.0.0.1:14550',
        description='MAVLink 连接目标，仿真默认连本机 SITL，真机部署时改成串口',
    )
    # 投放舵机参数（输出口 + 中立 PWM + 时长），见 docs/spec/adr/0007。默认值取机体固定配置
    # cuadc_physical/airframe.yaml 的 drop_servo（单一真源，各口 PWM 也在那里）；现场可用 drop_run_s:=… 临时覆盖。
    servo = cuadc_physical.load_airframe().drop_servo
    drop_servo_arg = DeclareLaunchArgument('drop_servo', default_value=str(servo.output))
    drop_neutral_us_arg = DeclareLaunchArgument('drop_neutral_us', default_value=str(servo.neutral_us))
    drop_run_s_arg = DeclareLaunchArgument('drop_run_s', default_value=str(servo.run_s))
    # 台架未解锁测舵机用；默认 false，飞行时不要开。
    allow_disarmed_drop_arg = DeclareLaunchArgument(
        'allow_disarmed_drop', default_value='false')
    # 仿真工具直接起本文件(不经 mission_bringup)时传 true，使遥测时间戳与 Gazebo 图像
    # 同在仿真时钟(拍照时刻对齐位姿需要,见 ADR-0010 B)。真机/默认 false 不受影响。
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    interface_node = Node(
        package='cuadc_interface',
        executable='interface_node',
        name='cuadc_interface',
        output='screen',
        parameters=[{
            'connection': LaunchConfiguration('connection'),
            'drop_servo': LaunchConfiguration('drop_servo'),
            'drop_neutral_us': LaunchConfiguration('drop_neutral_us'),
            'drop_run_s': LaunchConfiguration('drop_run_s'),
            'allow_disarmed_drop': ParameterValue(
                LaunchConfiguration('allow_disarmed_drop'), value_type=bool),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        connection_arg, drop_servo_arg,
        drop_neutral_us_arg, drop_run_s_arg, allow_disarmed_drop_arg,
        use_sim_time_arg, interface_node])
