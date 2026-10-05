"""地面站实时监测 dashboard：起 rosbridge（ROS2 话题<->浏览器 WebSocket
桥）+ web_video_server（压缩图像话题<->浏览器 MJPEG）+ 一个静态文件服务器
托管 web/ 下的前端页面。三者都是现成工具，这里只是组合，不写协议桥接
代码。纯订阅型旁路，不影响任务节点，可以和 mission_bringup.launch.py
同时跑也可以事后单独起来看 rosbag 之外的实时画面（回放另有 feature/
viz-replay 分支）。

依赖（不在 rosdep 默认基础安装里，需要单独装）：
    sudo apt-get install ros-humble-rosbridge-suite ros-humble-web-video-server
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port', default_value='9090', description='ROS2<->浏览器 WebSocket 桥端口')
    web_video_port_arg = DeclareLaunchArgument(
        'web_video_port', default_value='8080', description='压缩图像话题 MJPEG 服务端口')
    web_port_arg = DeclareLaunchArgument(
        'web_port', default_value='8000', description='前端静态页面 HTTP 端口')

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'])),
        launch_arguments={'port': LaunchConfiguration('rosbridge_port')}.items(),
    )

    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='viz_web_video_server',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('web_video_port'),
            # 无线链路上真正传的是 JPEG 压缩字节（image_transport 的
            # compressed 话题），不是原始像素——这是带宽预算能成立的关键，
            # 不是可选项，见 viz/README.md 的带宽设计一节。
            'default_transport': 'compressed',
        }],
    )

    static_web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', LaunchConfiguration('web_port'),
             '--directory', PathJoinSubstitution([FindPackageShare('viz'), 'web'])],
        output='screen',
    )

    # cuadc_perception_cv/interface 只发布原始 sensor_msgs/Image（rclpy 没有
    # image_transport 的 Python 绑定），压缩由这几个独立的 `republish`
    # 进程（image_transport_plugins 自带的标准工具）转，不用改节点代码。
    # web_video_server 上面已经配了 default_transport=compressed，找的就是
    # 这里产出的 <topic>/compressed。
    def _republish_node(base_topic):
        return Node(
            package='image_transport', executable='republish', output='screen',
            arguments=['raw', 'compressed'],
            remappings=[('in', base_topic), ('out/compressed', base_topic + '/compressed')],
        )

    republish_nodes = [
        _republish_node('/camera/image_raw'),
        _republish_node('/cuadc_perception_cv/debug/annotated'),
        _republish_node('/cuadc_perception_cv/debug/contours'),
        _republish_node('/cuadc_perception_cv/debug/mask'),
        _republish_node('/cuadc_perception_cv/recon_image'),
    ]

    return LaunchDescription([
        rosbridge_port_arg, web_video_port_arg, web_port_arg,
        rosbridge_launch, web_video_server_node, static_web_server,
        *republish_nodes,
    ])
