from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    competition_mode = LaunchConfiguration("competition_mode")
    config = os.path.join(
        get_package_share_directory("home_patrol"), "config", "waypoints.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "competition_mode",
                default_value="false",
                description="Manage microphone/arm/vision for the competition flow",
            ),
            Node(
                package="robot_mqtt_bridge",
                executable="unified_bridge",
                name="unified_mqtt_bridge",
                output="screen",
            ),
            Node(
                package="home_patrol",
                executable="patrol_node",
                name="home_patrol",
                output="screen",
                parameters=[{"waypoints_file": config}],
            ),
            Node(
                package="home_patrol",
                executable="map_http_server",
                name="home_map_http_server",
                output="screen",
                parameters=[{"port": 8090}],
            ),
            Node(
                package="home_patrol",
                executable="navigation_manager",
                name="home_navigation_manager",
                output="screen",
                parameters=[
                    {
                        "waypoints_file": config,
                        "competition_mode": ParameterValue(
                            competition_mode, value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
