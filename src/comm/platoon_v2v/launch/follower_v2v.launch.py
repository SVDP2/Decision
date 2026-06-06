import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('platoon_v2v')
    params = os.path.join(package_share, 'config', 'follower_v2v.yaml')
    enable_preview_path = LaunchConfiguration('enable_preview_path')
    enable_reference_path = LaunchConfiguration('enable_reference_path')
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_preview_path',
            default_value='false',
            description='Start legacy straight leader_preview_path_node.',
        ),
        DeclareLaunchArgument(
            'enable_reference_path',
            default_value='false',
            description='Adapt /v2v/leader/reference_path for follower lateral path tracking.',
        ),
        Node(
            package='platoon_v2v',
            executable='follower_v2v_receiver_node',
            name='follower_v2v_receiver_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='platoon_v2v',
            executable='leader_preview_path_node',
            name='leader_preview_path_node',
            output='screen',
            condition=IfCondition(enable_preview_path),
            parameters=[params],
        ),
        Node(
            package='platoon_v2v',
            executable='follower_reference_path_adapter_node',
            name='follower_reference_path_adapter_node',
            output='screen',
            condition=IfCondition(enable_reference_path),
            parameters=[params],
        ),
    ])
