"""起真实 CV 感知节点。跟 stub_perception_bringup.launch.py 平级，两者
发布同一份契约（见 docs/spec/05-perception-decision-interface.md），
决策层不需要区分对面是哪一个。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 相机内参 + 挂载角不在这里声明:按 profile 从 camera.<profile>.yaml 读(见 geometry.load_camera_params)。
    cylinder_height_arg = DeclareLaunchArgument(
        'cylinder_height_m', default_value='0.3', description='圆筒高度（米），反投影用')
    min_radius_arg = DeclareLaunchArgument(
        'min_radius_m', default_value='0.05', description='候选物理半径下限（米），过滤明显不合理的检测')
    max_radius_arg = DeclareLaunchArgument(
        'max_radius_m', default_value='0.18', description='候选物理半径上限（米）')
    profile_arg = DeclareLaunchArgument(
        'profile', default_value='sim',
        description='检测器的 sim/real 档：读 detectors/<detector>.<profile>.yaml，仿真 sim、真机 real')
    debug_annotated_arg = DeclareLaunchArgument(
        'debug_annotated_enabled', default_value='false', description='是否发布标注调试图，运行时可用 ros2 param set 切换')
    debug_contours_arg = DeclareLaunchArgument(
        'debug_contours_enabled', default_value='false', description='是否发布轮廓调试图')
    debug_mask_arg = DeclareLaunchArgument(
        'debug_mask_enabled', default_value='false', description='是否发布二值化 mask 调试图')
    max_concurrent_debug_arg = DeclareLaunchArgument(
        'max_concurrent_debug_views', default_value='4',
        description='同时最多算/发几路调试图像——真机上的硬性保护，跟前端请求了几路无关，见 cuadc_viz/README.md',
    )
    # 取帧源 + 真机预览(见 docs/spec/adr/0008-perception-raw-frame-multi-sink.md)。
    # 默认 sim 友好:frame_source=ros(订阅 image_raw)、预览关。真机 launch 传
    # frame_source:=shm preview_enabled:=true,图像就不进 ROS、标注走 RTP。
    frame_source_arg = DeclareLaunchArgument(
        'frame_source', default_value='ros',
        description='取帧源:sim=ros(订阅/camera/image_raw)、real=shm(读原始帧,图不进ROS)')
    raw_shm_arg = DeclareLaunchArgument('raw_shm', default_value='/tmp/cuadc-cam-raw.shm')
    src_width_arg = DeclareLaunchArgument('src_width', default_value='1920')
    src_height_arg = DeclareLaunchArgument('src_height', default_value='1080')
    src_fps_arg = DeclareLaunchArgument('src_fps', default_value='10')
    cv_width_arg = DeclareLaunchArgument('cv_width', default_value='960', description='shm 源出帧/检测/预览分辨率;真机内参要对应这个分辨率')
    cv_height_arg = DeclareLaunchArgument('cv_height', default_value='540')
    preview_enabled_arg = DeclareLaunchArgument('preview_enabled', default_value='false', description='真机把标注预览编码推本地 preview_port(cv-forward 转地面)')
    preview_port_arg = DeclareLaunchArgument('preview_port', default_value='5604')
    preview_bitrate_arg = DeclareLaunchArgument('preview_bitrate', default_value='1000')
    # 仿真工具直接起本文件(不经 mission_bringup)时传 true(见 interface_bringup 同名参数)。
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    # 量测/压测工具插图像延迟节点时把订阅 remap 到延迟后话题(默认=原话题,no-op)；
    # pose_at=latest 仅证据链 A/B / 真机拍照戳异常兜底用。
    image_topic_arg = DeclareLaunchArgument('image_topic', default_value='/camera/image_raw')
    pose_at_arg = DeclareLaunchArgument('pose_at', default_value='capture')

    cv_perception_node = Node(
        package='cuadc_perception',
        executable='cv_perception_node',
        name='cuadc_perception_cv',
        output='screen',
        parameters=[{
            'cylinder_height_m': LaunchConfiguration('cylinder_height_m'),
            'min_radius_m': LaunchConfiguration('min_radius_m'),
            'max_radius_m': LaunchConfiguration('max_radius_m'),
            'profile': LaunchConfiguration('profile'),
            'debug_annotated_enabled': LaunchConfiguration('debug_annotated_enabled'),
            'debug_contours_enabled': LaunchConfiguration('debug_contours_enabled'),
            'debug_mask_enabled': LaunchConfiguration('debug_mask_enabled'),
            'max_concurrent_debug_views': LaunchConfiguration('max_concurrent_debug_views'),
            'frame_source': LaunchConfiguration('frame_source'),
            'raw_shm': LaunchConfiguration('raw_shm'),
            'src_width': LaunchConfiguration('src_width'),
            'src_height': LaunchConfiguration('src_height'),
            'src_fps': LaunchConfiguration('src_fps'),
            'cv_width': LaunchConfiguration('cv_width'),
            'cv_height': LaunchConfiguration('cv_height'),
            'preview_enabled': LaunchConfiguration('preview_enabled'),
            'preview_port': LaunchConfiguration('preview_port'),
            'preview_bitrate': LaunchConfiguration('preview_bitrate'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'pose_at': LaunchConfiguration('pose_at'),
        }],
        remappings=[('/camera/image_raw', LaunchConfiguration('image_topic'))],
    )

    return LaunchDescription([
        cylinder_height_arg, min_radius_arg, max_radius_arg, profile_arg,
        debug_annotated_arg, debug_contours_arg, debug_mask_arg, max_concurrent_debug_arg,
        frame_source_arg, raw_shm_arg, src_width_arg, src_height_arg, src_fps_arg,
        cv_width_arg, cv_height_arg, preview_enabled_arg, preview_port_arg, preview_bitrate_arg,
        use_sim_time_arg, image_topic_arg, pose_at_arg,
        cv_perception_node,
    ])
