import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('platoon_localization')
    map_params = os.path.join(package_share, 'config', 'outdoor_utm_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('fix_topic', default_value='/f9p/fix'),
        DeclareLaunchArgument('fix_velocity_topic', default_value='/f9p/fix_velocity'),
        DeclareLaunchArgument('heading_topic', default_value='/vehicle_heading_rad'),
        DeclareLaunchArgument(
            'heading_valid_topic',
            default_value='/vehicle_heading_valid',
        ),
        DeclareLaunchArgument(
            'base_odom_topic',
            default_value='/leader/localization/gps/odom',
        ),
        DeclareLaunchArgument(
            'gps_odom_topic',
            default_value='/leader/localization/gps_antenna/odom',
        ),
        DeclareLaunchArgument('gps_frame', default_value='leader/leader_gps'),
        DeclareLaunchArgument('base_frame', default_value='leader/base_link'),
        DeclareLaunchArgument('gps_to_base_x_m', default_value='0.270'),
        DeclareLaunchArgument('gps_to_base_y_m', default_value='0.0'),
        DeclareLaunchArgument('gps_to_base_z_m', default_value='1.4675'),
        DeclareLaunchArgument('use_fixed_base_z', default_value='true'),
        DeclareLaunchArgument('fixed_base_z_m', default_value='0.1325'),
        DeclareLaunchArgument('publish_base_without_heading', default_value='true'),
        DeclareLaunchArgument('heading_yaw_variance_rad2', default_value='0.01'),
        DeclareLaunchArgument('unknown_heading_yaw_variance_rad2', default_value='9.869604401'),
        DeclareLaunchArgument('min_fix_status', default_value='2'),
        DeclareLaunchArgument('publish_base_tf', default_value='true'),
        Node(
            package='platoon_localization',
            executable='utm_offset_gps_odom_node',
            name='leader_utm_offset_gps_odom_node',
            output='screen',
            parameters=[
                map_params,
                {
                    'fix_topic': LaunchConfiguration('fix_topic'),
                    'fix_velocity_topic': LaunchConfiguration('fix_velocity_topic'),
                    'heading_topic': LaunchConfiguration('heading_topic'),
                    'heading_valid_topic': LaunchConfiguration('heading_valid_topic'),
                    'base_odom_topic': LaunchConfiguration('base_odom_topic'),
                    'gps_odom_topic': LaunchConfiguration('gps_odom_topic'),
                    'gps_frame': LaunchConfiguration('gps_frame'),
                    'base_frame': LaunchConfiguration('base_frame'),
                    'gps_to_base_x_m': ParameterValue(
                        LaunchConfiguration('gps_to_base_x_m'),
                        value_type=float,
                    ),
                    'gps_to_base_y_m': ParameterValue(
                        LaunchConfiguration('gps_to_base_y_m'),
                        value_type=float,
                    ),
                    'gps_to_base_z_m': ParameterValue(
                        LaunchConfiguration('gps_to_base_z_m'),
                        value_type=float,
                    ),
                    'use_fixed_base_z': ParameterValue(
                        LaunchConfiguration('use_fixed_base_z'),
                        value_type=bool,
                    ),
                    'fixed_base_z_m': ParameterValue(
                        LaunchConfiguration('fixed_base_z_m'),
                        value_type=float,
                    ),
                    'publish_base_without_heading': ParameterValue(
                        LaunchConfiguration('publish_base_without_heading'),
                        value_type=bool,
                    ),
                    'heading_yaw_variance_rad2': ParameterValue(
                        LaunchConfiguration('heading_yaw_variance_rad2'),
                        value_type=float,
                    ),
                    'unknown_heading_yaw_variance_rad2': ParameterValue(
                        LaunchConfiguration('unknown_heading_yaw_variance_rad2'),
                        value_type=float,
                    ),
                    'min_fix_status': ParameterValue(
                        LaunchConfiguration('min_fix_status'),
                        value_type=int,
                    ),
                    'publish_base_tf': ParameterValue(
                        LaunchConfiguration('publish_base_tf'),
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
