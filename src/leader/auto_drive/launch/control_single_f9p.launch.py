import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    auto_drive_share_dir = get_package_share_directory('auto_drive')
    mission_params = os.path.join(
        auto_drive_share_dir, 'config', 'mission_supervisor.yaml'
    )

    mission_supervisor_node = Node(
        package='auto_drive',
        executable='mission_supervisor_node',
        name='mission_supervisor_node',
        output='screen',
        parameters=[mission_params],
    )

    return LaunchDescription([mission_supervisor_node])
