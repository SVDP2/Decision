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
        DeclareLaunchArgument('fix_topic', default_value='/ublox_gps_node/fix'),
        DeclareLaunchArgument(
            'fix_velocity_topic',
            default_value='/ublox_gps_node/fix_velocity',
        ),
        DeclareLaunchArgument('heading_topic', default_value=''),
        DeclareLaunchArgument('heading_valid_topic', default_value=''),
        DeclareLaunchArgument(
            'gps_odom_topic',
            default_value='/follower/localization/gps/odom',
        ),
        DeclareLaunchArgument(
            'base_odom_topic',
            default_value='/follower/localization/global/odom',
        ),
        DeclareLaunchArgument('gps_frame', default_value='follower/follower_gps'),
        DeclareLaunchArgument('base_frame', default_value='follower/base_link'),
        DeclareLaunchArgument('gps_to_base_x_m', default_value='0.1625'),
        DeclareLaunchArgument('gps_to_base_y_m', default_value='0.0'),
        DeclareLaunchArgument('gps_to_base_z_m', default_value='0.135'),
        DeclareLaunchArgument('use_fixed_base_z', default_value='true'),
        DeclareLaunchArgument('fixed_base_z_m', default_value='0.055'),
        DeclareLaunchArgument('min_heading_speed_mps', default_value='0.5'),
        DeclareLaunchArgument('initial_heading_rad', default_value='0.0'),
        DeclareLaunchArgument('initial_heading_valid', default_value='false'),
        DeclareLaunchArgument('publish_base_without_heading', default_value='true'),
        DeclareLaunchArgument('heading_yaw_variance_rad2', default_value='0.01'),
        DeclareLaunchArgument('unknown_heading_yaw_variance_rad2', default_value='9.869604401'),
        DeclareLaunchArgument('min_fix_status', default_value='2'),
        DeclareLaunchArgument('publish_base_tf', default_value='true'),
        Node(
            package='platoon_localization',
            executable='utm_offset_gps_odom_node',
            name='follower_utm_offset_gps_odom_node',
            output='screen',
            parameters=[
                map_params,
                {
                    'fix_topic': LaunchConfiguration('fix_topic'),
                    'fix_velocity_topic': LaunchConfiguration('fix_velocity_topic'),
                    'heading_topic': LaunchConfiguration('heading_topic'),
                    'heading_valid_topic': LaunchConfiguration('heading_valid_topic'),
                    'gps_odom_topic': LaunchConfiguration('gps_odom_topic'),
                    'base_odom_topic': LaunchConfiguration('base_odom_topic'),
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
                    'min_heading_speed_mps': ParameterValue(
                        LaunchConfiguration('min_heading_speed_mps'),
                        value_type=float,
                    ),
                    'initial_heading_rad': ParameterValue(
                        LaunchConfiguration('initial_heading_rad'),
                        value_type=float,
                    ),
                    'initial_heading_valid': ParameterValue(
                        LaunchConfiguration('initial_heading_valid'),
                        value_type=bool,
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
