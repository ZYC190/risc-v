"""One-command startup with a persistent base and phone-selectable modes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package_name, launch_file, launch_arguments=None, condition=None):
    launch_path = os.path.join(
        get_package_share_directory(package_name), "launch", launch_file
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    enable_mobile_base = LaunchConfiguration("enable_mobile_base")
    auto_enable_interaction = LaunchConfiguration("auto_enable_interaction")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_mobile_base",
                default_value="true",
                description="Start the chassis/odometry stack when its hardware is connected",
            ),
            DeclareLaunchArgument(
                "auto_enable_interaction",
                default_value="false",
                description="Start live microphone and sound localization at boot",
            ),
            _include(
                "turn_on_wheeltec_robot",
                "turn_on_wheeltec_robot.launch.py",
                condition=IfCondition(enable_mobile_base),
            ),
            _include(
                "home_patrol",
                "home_patrol_system.launch.py",
                {
                    "competition_mode": "true",
                    "auto_enable_interaction": auto_enable_interaction,
                },
            ),
        ]
    )
