import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_CSV_PATH = (
    '/Users/yoosm/FinalProject/GP_Decision/config/path_csv/'
    'rosbag2_2026_03_14-16_23_49.csv'
)


def generate_launch_description():
    auto_drive_share_dir = get_package_share_directory('auto_drive')
    gps_to_utm_share_dir = get_package_share_directory('gps_to_utm')
    default_rviz_config = os.path.join(
        auto_drive_share_dir, 'config', 'single_f9p_debug.rviz'
    )

    csv_file_arg = DeclareLaunchArgument(
        'csv_file_path', default_value=DEFAULT_CSV_PATH
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz_config
    )
    use_serial_bridge_arg = DeclareLaunchArgument(
        'use_serial_bridge', default_value='false'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0'
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gps_to_utm_share_dir,
                'launch',
                'localization_single_f9p.launch.py',
            )
        ),
        launch_arguments={
            'csv_file_path': LaunchConfiguration('csv_file_path'),
        }.items(),
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                auto_drive_share_dir, 'launch', 'planning_single_f9p.launch.py'
            )
        )
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                auto_drive_share_dir, 'launch', 'control_single_f9p.launch.py'
            )
        )
    )

    serial_bridge_node = Node(
        package='serial_bridge',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        output='screen',
        parameters=[
            {
                'input_mode': 'direct',
                'port': LaunchConfiguration('serial_port'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('use_serial_bridge')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(
        [
            csv_file_arg,
            use_rviz_arg,
            rviz_config_arg,
            use_serial_bridge_arg,
            serial_port_arg,
            localization_launch,
            planning_launch,
            control_launch,
            serial_bridge_node,
            rviz_node,
        ]
    )
