from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _static_tf_node(
    name,
    parent_frame,
    child_frame,
    x,
    y,
    z,
    qx='0.0',
    qy='0.0',
    qz='0.0',
    qw='1.0',
):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        output='screen',
        arguments=[
            '--x', x,
            '--y', y,
            '--z', z,
            '--qx', qx,
            '--qy', qy,
            '--qz', qz,
            '--qw', qw,
            '--frame-id', parent_frame,
            '--child-frame-id', child_frame,
        ],
    )


def generate_launch_description() -> LaunchDescription:
    base_frame = LaunchConfiguration('base_frame')
    leader_gps_frame = LaunchConfiguration('leader_gps_frame')
    leader_rear_frame = LaunchConfiguration('leader_rear_frame')

    gps_x_m = LaunchConfiguration('gps_x_m')
    gps_y_m = LaunchConfiguration('gps_y_m')
    gps_z_m = LaunchConfiguration('gps_z_m')
    leader_rear_x_m = LaunchConfiguration('leader_rear_x_m')
    leader_rear_y_m = LaunchConfiguration('leader_rear_y_m')
    leader_rear_z_m = LaunchConfiguration('leader_rear_z_m')
    leader_rear_qx = LaunchConfiguration('leader_rear_qx')
    leader_rear_qy = LaunchConfiguration('leader_rear_qy')
    leader_rear_qz = LaunchConfiguration('leader_rear_qz')
    leader_rear_qw = LaunchConfiguration('leader_rear_qw')
    publish_vehicle_marker = LaunchConfiguration('publish_vehicle_marker')
    marker_topic = LaunchConfiguration('marker_topic')
    marker_publish_rate_hz = LaunchConfiguration('marker_publish_rate_hz')
    wheelbase_m = LaunchConfiguration('wheelbase_m')
    track_width_m = LaunchConfiguration('track_width_m')
    wheel_diameter_m = LaunchConfiguration('wheel_diameter_m')
    wheel_width_m = LaunchConfiguration('wheel_width_m')

    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='leader/base_link'),
        DeclareLaunchArgument('leader_gps_frame', default_value='leader/leader_gps'),
        DeclareLaunchArgument('leader_rear_frame', default_value='leader/leader_rear'),
        DeclareLaunchArgument('publish_vehicle_marker', default_value='true'),
        DeclareLaunchArgument('marker_topic', default_value='/leader/vehicle_model/markers'),
        DeclareLaunchArgument('marker_publish_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('wheelbase_m', default_value='0.720'),
        DeclareLaunchArgument('track_width_m', default_value='0.700'),
        DeclareLaunchArgument('wheel_diameter_m', default_value='0.265'),
        DeclareLaunchArgument('wheel_width_m', default_value='0.110'),
        # GPS antenna: 450 mm behind front axle, centered, 1.6 m above ground.
        # With base_link at rear axle center and axle height:
        # x = 0.720 - 0.450 = 0.270, z = 1.600 - 0.1325 = 1.4675.
        DeclareLaunchArgument('gps_x_m', default_value='0.270'),
        DeclareLaunchArgument('gps_y_m', default_value='0.000'),
        DeclareLaunchArgument('gps_z_m', default_value='1.4675'),
        # leader_rear is 545 mm behind the GPS x reference.
        # Marker top height 340 mm -> marker center 278 mm -> board origin 185 mm
        # above ground. base_link is axle height 132.5 mm above ground.
        DeclareLaunchArgument('leader_rear_x_m', default_value='-0.275'),
        DeclareLaunchArgument('leader_rear_y_m', default_value='0.000'),
        DeclareLaunchArgument('leader_rear_z_m', default_value='0.0525'),
        DeclareLaunchArgument('leader_rear_qx', default_value='0.0'),
        DeclareLaunchArgument('leader_rear_qy', default_value='0.0'),
        DeclareLaunchArgument('leader_rear_qz', default_value='0.0'),
        DeclareLaunchArgument('leader_rear_qw', default_value='1.0'),
        _static_tf_node(
            'leader_base_link_to_leader_gps_static_tf',
            base_frame,
            leader_gps_frame,
            gps_x_m,
            gps_y_m,
            gps_z_m,
        ),
        _static_tf_node(
            'leader_base_link_to_leader_rear_static_tf',
            base_frame,
            leader_rear_frame,
            leader_rear_x_m,
            leader_rear_y_m,
            leader_rear_z_m,
            leader_rear_qx,
            leader_rear_qy,
            leader_rear_qz,
            leader_rear_qw,
        ),
        Node(
            package='platoon_localization',
            executable='leader_vehicle_model_marker_node',
            name='leader_vehicle_model_marker_node',
            output='screen',
            condition=IfCondition(publish_vehicle_marker),
            parameters=[
                {
                    'base_frame': base_frame,
                    'marker_topic': marker_topic,
                    'publish_rate_hz': ParameterValue(
                        marker_publish_rate_hz,
                        value_type=float,
                    ),
                    'wheelbase_m': ParameterValue(wheelbase_m, value_type=float),
                    'track_width_m': ParameterValue(track_width_m, value_type=float),
                    'wheel_diameter_m': ParameterValue(
                        wheel_diameter_m,
                        value_type=float,
                    ),
                    'wheel_width_m': ParameterValue(wheel_width_m, value_type=float),
                    'gps_x_m': ParameterValue(gps_x_m, value_type=float),
                    'gps_y_m': ParameterValue(gps_y_m, value_type=float),
                    'gps_z_m': ParameterValue(gps_z_m, value_type=float),
                    'leader_rear_x_m': ParameterValue(
                        leader_rear_x_m,
                        value_type=float,
                    ),
                    'leader_rear_y_m': ParameterValue(
                        leader_rear_y_m,
                        value_type=float,
                    ),
                    'leader_rear_z_m': ParameterValue(
                        leader_rear_z_m,
                        value_type=float,
                    ),
                },
            ],
        ),
    ])
