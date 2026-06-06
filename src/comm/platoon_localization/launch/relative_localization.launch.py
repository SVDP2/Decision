import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    robot_namespace = LaunchConfiguration('robot_namespace')
    frame_prefix = LaunchConfiguration('frame_prefix')
    leader_frame_prefix = LaunchConfiguration('leader_frame_prefix')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_lidar = LaunchConfiguration('enable_lidar')
    video_device = LaunchConfiguration('video_device')
    aruco_odom_topic = LaunchConfiguration('aruco_odom_topic')
    leader_motion_topic = LaunchConfiguration('leader_motion_topic')
    leader_safety_topic = LaunchConfiguration('leader_safety_topic')
    heartbeat_topic = LaunchConfiguration('heartbeat_topic')
    output_topic = LaunchConfiguration('output_topic')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')

    relative_launch = os.path.join(
        get_package_share_directory('relative_localization_eskf'),
        'launch',
        'relative_localization.launch.py',
    )
    lidar_wheel_fitting_launch = os.path.join(
        get_package_share_directory('follower_lidar_localization'),
        'launch',
        'leader_wheel_fitting.launch.py',
    )
    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='follower'),
        DeclareLaunchArgument('frame_prefix', default_value='follower/'),
        DeclareLaunchArgument('leader_frame_prefix', default_value='leader/'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_imu', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument(
            'aruco_odom_topic',
            default_value='/follower/localization/leader_base/odom',
        ),
        DeclareLaunchArgument(
            'leader_motion_topic',
            default_value='/v2v/leader/motion_state',
        ),
        DeclareLaunchArgument(
            'leader_safety_topic',
            default_value='/v2v/leader/safety_state',
        ),
        DeclareLaunchArgument('heartbeat_topic', default_value='/v2v/leader/heartbeat'),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/platoon/relative_leader/state',
        ),
        DeclareLaunchArgument('publish_rate_hz', default_value='50.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(relative_launch),
            launch_arguments={
                'robot_namespace': robot_namespace,
                'frame_prefix': frame_prefix,
                'leader_frame_prefix': leader_frame_prefix,
                'enable_camera': enable_camera,
                'enable_imu': enable_imu,
                'enable_lidar': enable_lidar,
                'video_device': video_device,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_wheel_fitting_launch),
            launch_arguments={
                'include_lidar_driver': 'false',
                'publish_lidar_static_tf': 'false',
                'scan_topic': '/follower/scan',
                'base_frame': 'follower/base_link',
                'aruco_prior_topic': aruco_odom_topic,
                'leader_base_odom_topic': '/follower/localization/lidar_wheels/leader_base_detection',
                'leader_rear_odom_topic': '/follower/localization/lidar_wheels/odom',
                'marker_topic': '/follower/localization/lidar_wheels/markers',
                'diagnostics_topic': '/follower/localization/lidar_wheels/diagnostics',
                'wheelbase_m': '0.722',
                'track_width_m': '0.660',
                'wheel_radius_m': '0.115',
                'wheel_width_m': '0.100',
                'leader_rear_x_m': '-0.275',
                'leader_rear_y_m': '0.0',
                'leader_rear_z_m': '0.0525',
                'leader_rear_yaw_deg': '0.0',
            }.items(),
            condition=IfCondition(enable_lidar),
        ),
        Node(
            package='platoon_localization',
            executable='relative_aruco_leader_node',
            name='relative_aruco_leader_node',
            output='screen',
            parameters=[
                {
                    'aruco_odom_topic': aruco_odom_topic,
                    'leader_motion_topic': leader_motion_topic,
                    'leader_safety_topic': leader_safety_topic,
                    'heartbeat_topic': heartbeat_topic,
                    'output_topic': output_topic,
                    'base_frame': 'follower/base_link',
                    'expected_odom_frame': 'leader/base_link',
                    'expected_odom_child_frame': 'follower/base_link',
                    'target_frame': 'leader/base_link',
                    'source': 5,
                    'publish_rate_hz': ParameterValue(
                        publish_rate_hz,
                        value_type=float,
                    ),
                },
            ],
        ),
    ])
