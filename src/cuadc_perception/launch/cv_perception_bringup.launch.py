"""起真实 CV 感知节点。跟 stub_perception_bringup.launch.py 平级，两者
发布同一份契约（见 docs/spec/05-perception-decision-interface.md），
决策层不需要区分对面是哪一个。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fx_arg = DeclareLaunchArgument('fx', default_value='205.5', description='相机水平焦距（像素），默认是 sim 实测值')
    fy_arg = DeclareLaunchArgument('fy', default_value='205.5', description='相机垂直焦距（像素）')
    cx_arg = DeclareLaunchArgument('cx', default_value='320.0', description='主点 x（像素）')
    cy_arg = DeclareLaunchArgument('cy', default_value='240.0', description='主点 y（像素）')
    roll_arg = DeclareLaunchArgument(
        'camera_fixed_roll_rad', default_value='0.0',
        description='云台固定挂载 roll，需跟 gazebo_bringup.launch.py 同名参数一致',
    )
    pitch_arg = DeclareLaunchArgument(
        'camera_fixed_pitch_rad', default_value='1.5708',
        description='云台固定挂载 pitch，需跟 gazebo_bringup.launch.py 同名参数一致',
    )
    yaw_arg = DeclareLaunchArgument(
        'camera_fixed_yaw_rad', default_value='0.0',
        description='云台固定挂载 yaw，需跟 gazebo_bringup.launch.py 同名参数一致',
    )
    cylinder_height_arg = DeclareLaunchArgument(
        'cylinder_height_m', default_value='0.3', description='圆筒高度（米），反投影用')
    min_radius_arg = DeclareLaunchArgument(
        'min_radius_m', default_value='0.05', description='候选物理半径下限（米），过滤明显不合理的检测')
    max_radius_arg = DeclareLaunchArgument(
        'max_radius_m', default_value='0.18', description='候选物理半径上限（米）')
    morph_kernel_arg = DeclareLaunchArgument(
        'morph_kernel_px', default_value='5', description='形态学去噪核大小（像素）')
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

    cv_perception_node = Node(
        package='cuadc_perception',
        executable='cv_perception_node',
        name='cuadc_perception_cv',
        output='screen',
        parameters=[{
            'fx': LaunchConfiguration('fx'),
            'fy': LaunchConfiguration('fy'),
            'cx': LaunchConfiguration('cx'),
            'cy': LaunchConfiguration('cy'),
            'camera_fixed_roll_rad': LaunchConfiguration('camera_fixed_roll_rad'),
            'camera_fixed_pitch_rad': LaunchConfiguration('camera_fixed_pitch_rad'),
            'camera_fixed_yaw_rad': LaunchConfiguration('camera_fixed_yaw_rad'),
            'cylinder_height_m': LaunchConfiguration('cylinder_height_m'),
            'min_radius_m': LaunchConfiguration('min_radius_m'),
            'max_radius_m': LaunchConfiguration('max_radius_m'),
            'morph_kernel_px': LaunchConfiguration('morph_kernel_px'),
            'debug_annotated_enabled': LaunchConfiguration('debug_annotated_enabled'),
            'debug_contours_enabled': LaunchConfiguration('debug_contours_enabled'),
            'debug_mask_enabled': LaunchConfiguration('debug_mask_enabled'),
            'max_concurrent_debug_views': LaunchConfiguration('max_concurrent_debug_views'),
        }],
    )

    return LaunchDescription([
        fx_arg, fy_arg, cx_arg, cy_arg, roll_arg, pitch_arg, yaw_arg,
        cylinder_height_arg, min_radius_arg, max_radius_arg, morph_kernel_arg,
        debug_annotated_arg, debug_contours_arg, debug_mask_arg, max_concurrent_debug_arg,
        cv_perception_node,
    ])
