"""One-command startup with a persistent base and phone-selectable modes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(package_name, launch_file, launch_arguments=None):
    launch_path = os.path.join(
        get_package_share_directory(package_name), "launch", launch_file
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription(
        [
            _include("turn_on_wheeltec_robot", "turn_on_wheeltec_robot.launch.py"),
            _include(
                "home_patrol",
                "home_patrol_system.launch.py",
                {"competition_mode": "true"},
            ),
        ]
    )
