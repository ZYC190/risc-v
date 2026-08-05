"""Velocity smoothing and a standalone laser safety gate for manual SLAM."""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(file_path: Path) -> dict:
    with file_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"Invalid robot configuration: {file_path}")
    return config


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    param_file = LaunchConfiguration("params")
    safety_param_file = LaunchConfiguration("safety_params")

    robot_share = get_package_share_directory("turn_on_wheeltec_robot")
    hardware_config = os.path.join(
        robot_share, "config", "wheeltec_param.yaml"
    )
    car_mode = load_yaml(Path(hardware_config))["car_mode"]
    nav_share = get_package_share_directory("wheeltec_nav2")
    default_param_file = os.path.join(
        nav_share, "param", "wheeltec_params", f"param_{car_mode}.yaml"
    )
    default_safety_file = os.path.join(
        nav_share, "param", "wheeltec_params", "mapping_safety.yaml"
    )
    time_parameter = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "params",
                default_value=default_param_file,
                description="Base Nav2 parameters for the configured chassis",
            ),
            DeclareLaunchArgument(
                "safety_params",
                default_value=default_safety_file,
                description="Manual mapping speed and collision overrides",
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[param_file, safety_param_file, time_parameter],
                remappings=[
                    ("cmd_vel", "cmd_vel_nav"),
                    ("cmd_vel_smoothed", "cmd_vel_raw"),
                ],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[param_file, safety_param_file, time_parameter],
                arguments=["--ros-args", "--log-level", "info"],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_mapping_safety",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        "autostart": ParameterValue(
                            autostart, value_type=bool
                        ),
                        "bond_timeout": 15.0,
                        "node_names": [
                            "velocity_smoother",
                            "collision_monitor",
                        ],
                    }
                ],
            ),
        ]
    )
