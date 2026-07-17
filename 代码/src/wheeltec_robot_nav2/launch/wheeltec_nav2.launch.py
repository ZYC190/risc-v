import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(file_path: Path) -> dict:
    with open(file_path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    start_base = LaunchConfiguration("start_base")
    start_lidar = LaunchConfiguration("start_lidar")
    start_waypoint_cycle = LaunchConfiguration("start_waypoint_cycle")

    wheeltec_robot_dir = get_package_share_directory("turn_on_wheeltec_robot")
    wheeltec_launch_dir = os.path.join(wheeltec_robot_dir, "launch")

    wheeltec_nav_dir = get_package_share_directory("wheeltec_nav2")
    wheeltec_nav_launch_dir = os.path.join(wheeltec_nav_dir, "launch")
    cfg_params = load_yaml(
        os.path.join(
            wheeltec_robot_dir,
            "config",
            "wheeltec_param.yaml",
        )
    )
    car_mode = cfg_params["car_mode"]

    map_dir = os.path.join(wheeltec_nav_dir, "map")
    default_map_file = os.path.join(map_dir, "WHEELTEC.yaml")
    map_file = LaunchConfiguration("map")

    param_dir = os.path.join(wheeltec_nav_dir, "param", "wheeltec_params")
    default_param_file = os.path.join(param_dir, f"param_{car_mode}.yaml")
    param_file = LaunchConfiguration("params")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=default_map_file,
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "params",
                default_value=default_param_file,
                description="Full path to Nav2 parameter file",
            ),
            DeclareLaunchArgument(
                "start_base",
                default_value="true",
                description="Start chassis nodes; set false when already running",
            ),
            DeclareLaunchArgument(
                "start_lidar",
                default_value="true",
                description="Start lidar nodes; set false when already running",
            ),
            DeclareLaunchArgument(
                "start_waypoint_cycle",
                default_value="true",
                description="Start the keyboard waypoint-cycle helper",
            ),
            Node(
                name="waypoint_cycle",
                package="nav2_waypoint_cycle",
                executable="nav2_waypoint_cycle",
                prefix="taskset -c 6 ",
                condition=IfCondition(start_waypoint_cycle),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        wheeltec_launch_dir, "turn_on_wheeltec_robot.launch.py"
                    )
                ),
                condition=IfCondition(start_base),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(wheeltec_launch_dir, "wheeltec_lidar.launch.py")
                ),
                condition=IfCondition(start_lidar),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(wheeltec_nav_launch_dir, "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": use_sim_time,
                    "params_file": param_file,
                }.items(),
            ),
        ]
    )
