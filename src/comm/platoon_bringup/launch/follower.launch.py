import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable


def generate_launch_description() -> LaunchDescription:
    bringup_launch = os.path.join(
        get_package_share_directory('platoon_bringup'),
        'launch',
        'follower_bringup.launch.py',
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '20'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable(
            'CYCLONEDDS_URI',
            [
                'file://',
                EnvironmentVariable('HOME'),
                '/.ros/cyclonedds_internal_loopback.xml',
            ],
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_launch)),
    ])
