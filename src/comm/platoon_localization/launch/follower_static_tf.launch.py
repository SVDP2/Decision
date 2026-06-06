from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Vehicle frame convention: FLU (x forward, y left, z up)
# base_link: rear axle center at wheel axle height.
TRACK_WIDTH_M = 0.235
WHEEL_BASE_M = 0.325
WHEEL_RADIUS_M = 0.055
HALF_TRACK_M = TRACK_WIDTH_M * 0.5
USB_CAM_OPTICAL_QUAT_XYZW = (-0.5, 0.5, -0.5, 0.5)

TRANSFORMS = (
    {
        'name': 'follower_base_link_to_base_imu_link_static_tf',
        'parent': 'base_link',
        'child': 'base_imu_link',
        'translation': (0.060, 0.0, 0.150 - WHEEL_RADIUS_M),
        'quaternion': (1.0, 0.0, 0.0, 0.0),
    },
    {
        'name': 'follower_base_link_to_usb_cam_static_tf',
        'parent': 'base_link',
        'child': 'usb_cam',
        'translation': (0.270, 0.0, 0.190 - WHEEL_RADIUS_M),
        'quaternion': USB_CAM_OPTICAL_QUAT_XYZW,
    },
    {
        'name': 'follower_usb_cam_to_lidar_static_tf',
        'parent': 'usb_cam',
        'child': 'lidar',
        'translation': (
            0.0025602837540221954,
            0.12819786529006177,
            0.133867715594006,
        ),
        'quaternion': (
            -0.17415269402591296,
            -0.6768985720658026,
            0.6965430734104255,
            -0.16219404792640454,
        ),
    },
    {
        'name': 'follower_base_link_to_follower_gps_static_tf',
        'parent': 'base_link',
        'child': 'follower_gps',
        'translation': (WHEEL_BASE_M * 0.5, 0.0, 0.190 - WHEEL_RADIUS_M),
        'quaternion': (0.0, 0.0, 0.0, 1.0),
    },
)


def _prefixed_frame(frame_prefix, frame):
    return PythonExpression(["'", frame_prefix, "' + '", frame, "'"])


def _static_tf_node(spec, frame_prefix):
    tx, ty, tz = spec['translation']
    qx, qy, qz, qw = spec['quaternion']
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=spec['name'],
        output='screen',
        arguments=[
            '--x', str(tx),
            '--y', str(ty),
            '--z', str(tz),
            '--qx', str(qx),
            '--qy', str(qy),
            '--qz', str(qz),
            '--qw', str(qw),
            '--frame-id', _prefixed_frame(frame_prefix, spec['parent']),
            '--child-frame-id', _prefixed_frame(frame_prefix, spec['child']),
        ],
    )


def generate_launch_description() -> LaunchDescription:
    frame_prefix = LaunchConfiguration('frame_prefix')
    publish_vehicle_marker = LaunchConfiguration('publish_vehicle_marker')
    marker_topic = LaunchConfiguration('marker_topic')
    marker_publish_rate_hz = LaunchConfiguration('marker_publish_rate_hz')
    wheelbase_m = LaunchConfiguration('wheelbase_m')
    track_width_m = LaunchConfiguration('track_width_m')
    wheel_diameter_m = LaunchConfiguration('wheel_diameter_m')
    wheel_width_m = LaunchConfiguration('wheel_width_m')
    return LaunchDescription([
        DeclareLaunchArgument('frame_prefix', default_value='follower/'),
        DeclareLaunchArgument('publish_vehicle_marker', default_value='true'),
        DeclareLaunchArgument('marker_topic', default_value='/follower/vehicle_model/markers'),
        DeclareLaunchArgument('marker_publish_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('wheelbase_m', default_value=str(WHEEL_BASE_M)),
        DeclareLaunchArgument('track_width_m', default_value=str(TRACK_WIDTH_M)),
        DeclareLaunchArgument('wheel_diameter_m', default_value=str(2.0 * WHEEL_RADIUS_M)),
        DeclareLaunchArgument('wheel_width_m', default_value='0.040'),
        *[_static_tf_node(spec, frame_prefix) for spec in TRANSFORMS],
        Node(
            package='platoon_localization',
            executable='leader_vehicle_model_marker_node',
            name='follower_vehicle_model_marker_node',
            output='screen',
            condition=IfCondition(publish_vehicle_marker),
            parameters=[
                {
                    'base_frame': ParameterValue(
                        _prefixed_frame(frame_prefix, 'base_link'),
                        value_type=str,
                    ),
                    'marker_topic': marker_topic,
                    'marker_namespace': 'follower_vehicle_model',
                    'publish_rate_hz': ParameterValue(
                        marker_publish_rate_hz,
                        value_type=float,
                    ),
                    'show_rear_reference': False,
                    'wheelbase_m': ParameterValue(wheelbase_m, value_type=float),
                    'track_width_m': ParameterValue(track_width_m, value_type=float),
                    'wheel_diameter_m': ParameterValue(
                        wheel_diameter_m,
                        value_type=float,
                    ),
                    'wheel_width_m': ParameterValue(wheel_width_m, value_type=float),
                    'gps_x_m': WHEEL_BASE_M * 0.5,
                    'gps_y_m': 0.0,
                    'gps_z_m': 0.190 - WHEEL_RADIUS_M,
                },
            ],
        ),
    ])
