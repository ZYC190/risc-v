from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("home_patrol"), "config", "waypoints.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="home_patrol",
                executable="patrol_node",
                name="home_patrol",
                output="screen",
                parameters=[{"waypoints_file": config}],
            )
        ]
    )
