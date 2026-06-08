import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_private_yaml() -> str:
    candidates = [
        os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "config", "ntrip_private.yaml")
        ),
        os.path.join(os.getcwd(), "um980_driver", "config", "ntrip_private.yaml"),
        os.path.join(os.getcwd(), "src", "INS", "um980_driver", "config", "ntrip_private.yaml"),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[0]


def generate_launch_description():
    share_dir = get_package_share_directory("um980_driver")
    default_config = os.path.join(share_dir, "config", "um980_50hz.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("ntrip_private_yaml", default_value=_default_private_yaml()),
        DeclareLaunchArgument("configure_on_start", default_value="true"),
        DeclareLaunchArgument("save_config", default_value="false"),
        DeclareLaunchArgument("mode", default_value="survey_50hz"),
        DeclareLaunchArgument("base_station_name", default_value=""),
        Node(
            package="um980_driver",
            executable="um980_driver_node",
            name="um980_driver",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                {
                    "configure_on_start": LaunchConfiguration("configure_on_start"),
                    "save_config": LaunchConfiguration("save_config"),
                    "mode": LaunchConfiguration("mode"),
                    "base_station_name": LaunchConfiguration("base_station_name"),
                    "ntrip_enabled": True,
                    "ntrip_private_yaml": LaunchConfiguration("ntrip_private_yaml"),
                },
            ],
        ),
    ])
