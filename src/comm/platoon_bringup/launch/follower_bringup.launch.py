import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_file(package_name, *parts):
    return os.path.join(get_package_share_directory(package_name), 'launch', *parts)


def generate_launch_description() -> LaunchDescription:
    enable_static_tf = LaunchConfiguration('enable_static_tf')
    enable_v2v = LaunchConfiguration('enable_v2v')
    enable_fused_relative_eskf = LaunchConfiguration('enable_fused_relative_eskf')
    enable_direct_gps_relative = LaunchConfiguration('enable_direct_gps_relative')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_follower_rtk_gps_odom = LaunchConfiguration('enable_follower_rtk_gps_odom')
    enable_motor_control = LaunchConfiguration('enable_motor_control')
    enable_lateral_control = LaunchConfiguration('enable_lateral_control')

    return LaunchDescription([
        DeclareLaunchArgument('enable_static_tf', default_value='true'),
        DeclareLaunchArgument('enable_v2v', default_value='true'),
        DeclareLaunchArgument('enable_fused_relative_eskf', default_value='true'),
        DeclareLaunchArgument('enable_direct_gps_relative', default_value='false'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_follower_rtk_gps_odom', default_value='true'),
        DeclareLaunchArgument('enable_motor_control', default_value='true'),
        DeclareLaunchArgument('enable_lateral_control', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('platoon_localization', 'follower_static_tf.launch.py')
            ),
            condition=IfCondition(enable_static_tf),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('platoon_v2v', 'follower_v2v.launch.py')
            ),
            condition=IfCondition(enable_v2v),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('platoon_localization', 'relative_localization.launch.py')
            ),
            launch_arguments={
                'enable_lidar': enable_lidar,
            }.items(),
            condition=IfCondition(enable_fused_relative_eskf),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file(
                    'platoon_localization',
                    'outdoor_relative_localization.launch.py',
                )
            ),
            condition=IfCondition(enable_direct_gps_relative),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('platoon_localization', 'follower_gps_odom.launch.py')
            ),
            launch_arguments={'publish_base_tf': 'false'}.items(),
            condition=IfCondition(enable_follower_rtk_gps_odom),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('xycar_motor_native', 'xycar_drive.launch.py')
            ),
            condition=IfCondition(enable_motor_control),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file(
                    'xycar_longitudinal_controller',
                    'signed_sync_longitudinal_controller.launch.py',
                )
            ),
            condition=IfCondition(enable_motor_control),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('xycar_lateral_controller', 'lateral_controller.launch.py')
            ),
            condition=IfCondition(enable_lateral_control),
        ),
    ])
