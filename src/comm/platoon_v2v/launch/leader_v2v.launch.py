import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('platoon_v2v')
    params = os.path.join(package_share, 'config', 'leader_v2v.yaml')
    enable_reference_path = LaunchConfiguration('enable_reference_path')
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_reference_path',
            default_value='true',
            description='Relay leader /roi_path to /v2v/leader/reference_path.',
        ),
        Node(
            package='platoon_v2v',
            executable='leader_v2v_adapter_node',
            name='leader_v2v_adapter_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='platoon_v2v',
            executable='leader_reference_path_relay_node',
            name='leader_reference_path_relay_node',
            output='screen',
            condition=IfCondition(enable_reference_path),
            parameters=[params],
        ),
    ])
