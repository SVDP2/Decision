import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration


def _launch_file(package_name, *parts):
    return os.path.join(get_package_share_directory(package_name), 'launch', *parts)


def generate_launch_description() -> LaunchDescription:
    enable_reference_path = LaunchConfiguration('enable_reference_path')
    enable_bridge = LaunchConfiguration('enable_bridge')
    leader_ip = LaunchConfiguration('leader_ip')
    follower_ip = LaunchConfiguration('follower_ip')
    wifi_interface = LaunchConfiguration('wifi_interface')

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '10'),
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
        DeclareLaunchArgument('enable_reference_path', default_value='true'),
        DeclareLaunchArgument('enable_bridge', default_value='true'),
        DeclareLaunchArgument('leader_ip', default_value='192.168.0.162'),
        DeclareLaunchArgument('follower_ip', default_value='192.168.0.113'),
        DeclareLaunchArgument('wifi_interface', default_value='wlp3s0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_file('platoon_v2v', 'leader_v2v.launch.py')
            ),
            launch_arguments={'enable_reference_path': enable_reference_path}.items(),
        ),
        ExecuteProcess(
            cmd=[
                'ros2',
                'run',
                'platoon_bringup',
                'svdp_leader_bridge',
                '--leader-ip',
                leader_ip,
                '--follower-ip',
                follower_ip,
                '--wifi-interface',
                wifi_interface,
            ],
            output='screen',
            condition=IfCondition(enable_bridge),
        ),
    ])
