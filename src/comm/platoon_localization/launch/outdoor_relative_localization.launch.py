from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    leader_odom_topic = LaunchConfiguration('leader_odom_topic')
    follower_odom_topic = LaunchConfiguration('follower_odom_topic')
    leader_motion_topic = LaunchConfiguration('leader_motion_topic')
    leader_safety_topic = LaunchConfiguration('leader_safety_topic')
    heartbeat_topic = LaunchConfiguration('heartbeat_topic')
    output_topic = LaunchConfiguration('output_topic')
    base_frame = LaunchConfiguration('base_frame')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument('leader_odom_topic', default_value='/v2v/leader/odom'),
        DeclareLaunchArgument(
            'follower_odom_topic',
            default_value='/follower/localization/global/odom',
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
        DeclareLaunchArgument('base_frame', default_value='follower/base_link'),
        DeclareLaunchArgument(
            'expected_leader_child_frame',
            default_value='leader/base_link',
        ),
        DeclareLaunchArgument(
            'expected_follower_child_frame',
            default_value='follower/base_link',
        ),
        DeclareLaunchArgument('publish_rate_hz', default_value='50.0'),
        Node(
            package='platoon_localization',
            executable='relative_gps_leader_node',
            name='relative_gps_leader_node',
            output='screen',
            parameters=[
                {
                    'leader_odom_topic': leader_odom_topic,
                    'follower_odom_topic': follower_odom_topic,
                    'leader_motion_topic': leader_motion_topic,
                    'leader_safety_topic': leader_safety_topic,
                    'heartbeat_topic': heartbeat_topic,
                    'output_topic': output_topic,
                    'base_frame': base_frame,
                    'expected_leader_child_frame': LaunchConfiguration(
                        'expected_leader_child_frame',
                    ),
                    'expected_follower_child_frame': LaunchConfiguration(
                        'expected_follower_child_frame',
                    ),
                    'target_frame': 'leader/base_link',
                    'publish_rate_hz': ParameterValue(
                        publish_rate_hz,
                        value_type=float,
                    ),
                },
            ],
        ),
    ])
