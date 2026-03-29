import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    auto_drive_share_dir = get_package_share_directory('auto_drive')
    roi_params = os.path.join(auto_drive_share_dir, 'config', 'roi_path.yaml')
    pure_pursuit_params = os.path.join(
        auto_drive_share_dir, 'config', 'pure_pursuit.yaml'
    )

    roi_path_node = Node(
        package='auto_drive',
        executable='roi_path_node',
        name='roi_path_node',
        output='screen',
        parameters=[roi_params],
    )

    pure_pursuit_node = Node(
        package='auto_drive',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[pure_pursuit_params],
    )

    return LaunchDescription([roi_path_node, pure_pursuit_node])
