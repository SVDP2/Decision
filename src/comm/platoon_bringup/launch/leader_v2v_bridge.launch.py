from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    leader_ip = LaunchConfiguration('leader_ip')
    follower_ip = LaunchConfiguration('follower_ip')
    wifi_interface = LaunchConfiguration('wifi_interface')

    return LaunchDescription([
        DeclareLaunchArgument('leader_ip', default_value='192.168.0.162'),
        DeclareLaunchArgument('follower_ip', default_value='192.168.0.113'),
        DeclareLaunchArgument('wifi_interface', default_value='wlp3s0'),
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
        ),
    ])
