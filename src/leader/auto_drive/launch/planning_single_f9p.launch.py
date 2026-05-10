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
    complex_target_params = os.path.join(
        auto_drive_share_dir, 'config', 'complex_target.yaml'
    )
    complex_rrt_params = os.path.join(
        auto_drive_share_dir, 'config', 'complex_rrt.yaml'
    )
    complex_pure_pursuit_params = os.path.join(
        auto_drive_share_dir, 'config', 'complex_pure_pursuit.yaml'
    )
    command_mux_params = os.path.join(
        auto_drive_share_dir, 'config', 'command_mux.yaml'
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
        parameters=[
            pure_pursuit_params,
            {
                'steer_topic': '/highway/auto_steer_angle',
                'throttle_topic': '/highway/throttle_from_planning',
            },
        ],
    )

    complex_target_node = Node(
        package='auto_drive',
        executable='complex_target_node',
        name='complex_target_node',
        output='screen',
        parameters=[complex_target_params],
    )

    complex_rrt_planner_node = Node(
        package='auto_drive',
        executable='complex_rrt_planner_node',
        name='complex_rrt_planner_node',
        output='screen',
        parameters=[complex_rrt_params],
    )

    complex_pure_pursuit_node = Node(
        package='auto_drive',
        executable='pure_pursuit_node',
        name='complex_pure_pursuit_node',
        output='screen',
        parameters=[complex_pure_pursuit_params],
    )

    command_mux_node = Node(
        package='auto_drive',
        executable='command_mux_node',
        name='command_mux_node',
        output='screen',
        parameters=[command_mux_params],
    )

    return LaunchDescription(
        [
            roi_path_node,
            pure_pursuit_node,
            complex_target_node,
            complex_rrt_planner_node,
            complex_pure_pursuit_node,
            command_mux_node,
        ]
    )
