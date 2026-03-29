import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_CSV_PATH = (
    '/Users/yoosm/FinalProject/GP_Decision/config/path_csv/'
    'rosbag2_2026_03_14-16_23_49.csv'
)


def generate_launch_description():
    gps_to_utm_share_dir = get_package_share_directory('gps_to_utm')
    heading_params = os.path.join(
        gps_to_utm_share_dir, 'config', 'single_f9p_heading.yaml'
    )
    tf_params = os.path.join(
        gps_to_utm_share_dir, 'config', 'tf_gps_csv_single.yaml'
    )

    csv_file_arg = DeclareLaunchArgument(
        'csv_file_path', default_value=DEFAULT_CSV_PATH
    )

    f9p_to_utm_node = Node(
        package='gps_to_utm',
        executable='f9p_to_utm',
        name='f9p_to_utm',
        output='screen',
    )

    single_heading_node = Node(
        package='gps_to_utm',
        executable='single_f9p_heading_node',
        name='single_f9p_heading_node',
        output='screen',
        parameters=[
            heading_params,
            {'csv_file_path': LaunchConfiguration('csv_file_path')},
        ],
    )

    tf_gps_csv_single_node = Node(
        package='gps_to_utm',
        executable='tf_gps_csv_single_node',
        name='tf_gps_csv_single_node',
        output='screen',
        parameters=[
            tf_params,
            {'csv_file_path': LaunchConfiguration('csv_file_path')},
        ],
    )

    return LaunchDescription([
        csv_file_arg,
        f9p_to_utm_node,
        single_heading_node,
        tf_gps_csv_single_node,
    ])
