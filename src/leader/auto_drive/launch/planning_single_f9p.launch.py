import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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
    mission_zone_params = os.path.join(
        auto_drive_share_dir, 'config', 'mission_zones.yaml'
    )

    publish_velodyne_tf = LaunchConfiguration('publish_velodyne_tf')
    velodyne_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='vehicle_ref_to_velodyne_tf',
        output='screen',
        condition=IfCondition(publish_velodyne_tf),
        arguments=[
            LaunchConfiguration('velodyne_x'),
            LaunchConfiguration('velodyne_y'),
            LaunchConfiguration('velodyne_z'),
            LaunchConfiguration('velodyne_yaw'),
            LaunchConfiguration('velodyne_pitch'),
            LaunchConfiguration('velodyne_roll'),
            LaunchConfiguration('vehicle_frame'),
            LaunchConfiguration('velodyne_frame'),
        ],
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

    mission_zone_node = Node(
        package='auto_drive',
        executable='mission_zone_node',
        name='mission_zone_node',
        output='screen',
        parameters=[
            mission_zone_params,
            {'csv_file_path': LaunchConfiguration('csv_file_path')},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'csv_file_path',
                default_value='',
                description='UTM CSV path used for mission zone csv_index/range.',
            ),
            DeclareLaunchArgument(
                'publish_velodyne_tf',
                default_value='true',
                description='Publish static TF vehicle_frame -> velodyne_frame.',
            ),
            DeclareLaunchArgument(
                'vehicle_frame',
                default_value='vehicle_ref',
                description='Vehicle planning frame used as static TF parent.',
            ),
            DeclareLaunchArgument(
                'velodyne_frame',
                default_value='velodyne',
                description='LiDAR frame used by detected object markers.',
            ),
            DeclareLaunchArgument(
                'velodyne_x',
                default_value='0.0',
                description='LiDAR x offset from vehicle_frame in meters.',
            ),
            DeclareLaunchArgument(
                'velodyne_y',
                default_value='0.0',
                description='LiDAR y offset from vehicle_frame in meters.',
            ),
            DeclareLaunchArgument(
                'velodyne_z',
                default_value='0.0',
                description='LiDAR z offset from vehicle_frame in meters.',
            ),
            DeclareLaunchArgument(
                'velodyne_yaw',
                default_value='0.0',
                description='LiDAR yaw offset from vehicle_frame in radians.',
            ),
            DeclareLaunchArgument(
                'velodyne_pitch',
                default_value='0.0',
                description='LiDAR pitch offset from vehicle_frame in radians.',
            ),
            DeclareLaunchArgument(
                'velodyne_roll',
                default_value='0.0',
                description='LiDAR roll offset from vehicle_frame in radians.',
            ),
            velodyne_tf_node,
            roi_path_node,
            pure_pursuit_node,
            complex_target_node,
            complex_rrt_planner_node,
            complex_pure_pursuit_node,
            command_mux_node,
            mission_zone_node,
        ]
    )
