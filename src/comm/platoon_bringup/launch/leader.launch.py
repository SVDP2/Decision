import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration


def _launch_file(package_name, *parts):
    return os.path.join(get_package_share_directory(package_name), 'launch', *parts)


def generate_launch_description() -> LaunchDescription:
    enable_static_tf = LaunchConfiguration('enable_static_tf')
    enable_leader_rtk_gps_odom = LaunchConfiguration('enable_leader_rtk_gps_odom')

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '10'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable(
            'CYCLONEDDS_URI',
            ['file://', EnvironmentVariable('HOME'), '/.ros/cyclonedds_internal_loopback.xml'],
        ),
        DeclareLaunchArgument('enable_static_tf', default_value='true'),
        DeclareLaunchArgument('enable_leader_rtk_gps_odom', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file('platoon_localization', 'leader_static_tf.launch.py')),
            condition=IfCondition(enable_static_tf),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file('platoon_localization', 'leader_gps_odom.launch.py')),
            launch_arguments={'publish_base_tf': 'false'}.items(),
            condition=IfCondition(enable_leader_rtk_gps_odom),
        ),
    ])
