"""Run gmapping and the protected Nav2 command chain on the live map.

This is the RPLIDAR C1-safe counterpart of the launch name used by the K1
reference guide.  It deliberately reuses navigation_launch_competition.py so
velocity smoothing and collision monitoring remain in the command path.
"""

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
    with file_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"Invalid robot configuration: {file_path}")
    return config


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_base = LaunchConfiguration("start_base")
    start_lidar = LaunchConfiguration("start_lidar")
    start_waypoint_cycle = LaunchConfiguration("start_waypoint_cycle")
    lidar_type = LaunchConfiguration("lidar_type")
    lidar_type_yaml = LaunchConfiguration("lidar_type_yaml")
    param_file = LaunchConfiguration("params")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    robot_share = get_package_share_directory("turn_on_wheeltec_robot")
    robot_launch_dir = os.path.join(robot_share, "launch")
    default_hardware_config = os.path.join(
        robot_share, "config", "wheeltec_param.yaml"
    )
    car_mode = load_yaml(Path(default_hardware_config))["car_mode"]

    nav_share = get_package_share_directory("wheeltec_nav2")
    nav_launch_dir = os.path.join(nav_share, "launch")
    default_param_file = os.path.join(
        nav_share, "param", "wheeltec_params", f"param_{car_mode}.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_respawn", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="warn"),
            DeclareLaunchArgument(
                "params",
                default_value=default_param_file,
                description="Nav2 parameters for the configured chassis",
            ),
            DeclareLaunchArgument(
                "start_base",
                default_value="true",
                description="Start chassis and odometry nodes",
            ),
            DeclareLaunchArgument(
                "start_lidar",
                default_value="true",
                description="Start the configured lidar driver",
            ),
            DeclareLaunchArgument(
                "start_waypoint_cycle",
                default_value="true",
                description="Start the keyboard waypoint-cycle helper",
            ),
            DeclareLaunchArgument(
                "lidar_type",
                default_value="",
                description="Override lidar model from the hardware YAML",
            ),
            DeclareLaunchArgument(
                "lidar_type_yaml",
                default_value=default_hardware_config,
                description="Robot and lidar hardware configuration YAML",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        robot_launch_dir, "turn_on_wheeltec_robot.launch.py"
                    )
                ),
                condition=IfCondition(start_base),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_launch_dir, "wheeltec_lidar.launch.py")
                ),
                launch_arguments={
                    "lidar_type": lidar_type,
                    "lidar_type_yaml": lidar_type_yaml,
                }.items(),
                condition=IfCondition(start_lidar),
            ),
            Node(
                package="slam_gmapping",
                executable="slam_gmapping",
                name="slam_gmapping",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        nav_launch_dir, "navigation_launch_competition.py"
                    )
                ),
                launch_arguments={
                    "namespace": "",
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": param_file,
                    "use_composition": "False",
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                    "log_level": log_level,
                }.items(),
            ),
            Node(
                package="nav2_waypoint_cycle",
                executable="nav2_waypoint_cycle",
                name="waypoint_cycle",
                output="screen",
                prefix="taskset -c 6 ",
                condition=IfCondition(start_waypoint_cycle),
            ),
        ]
    )
