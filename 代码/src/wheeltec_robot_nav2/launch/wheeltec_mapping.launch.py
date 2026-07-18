"""Hardware-safe mapping wrapper for gmapping or Cartographer."""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def load_yaml(file_path: Path) -> dict:
    with file_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"Invalid robot configuration: {file_path}")
    return config


def include_mapper(context, *args, **kwargs):
    del args, kwargs
    mapper = LaunchConfiguration("mapper").perform(context).strip().lower()
    launch_files = {
        "gmapping": ("slam_gmapping", "slam_gmapping.launch.py"),
        "cartographer": (
            "wheeltec_cartographer",
            "cartographer.launch.py",
        ),
    }
    if mapper not in launch_files:
        raise ValueError("mapper must be 'gmapping' or 'cartographer'")
    package_name, launch_file = launch_files[mapper]
    launch_path = os.path.join(
        get_package_share_directory(package_name), "launch", launch_file
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "start_base": LaunchConfiguration("start_base"),
                "start_lidar": LaunchConfiguration("start_lidar"),
                "lidar_type": LaunchConfiguration("lidar_type"),
                "lidar_type_yaml": LaunchConfiguration("lidar_type_yaml"),
            }.items(),
        )
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_safety = LaunchConfiguration("start_safety")
    param_file = LaunchConfiguration("params")

    robot_share = get_package_share_directory("turn_on_wheeltec_robot")
    hardware_config = os.path.join(
        robot_share, "config", "wheeltec_param.yaml"
    )
    car_mode = load_yaml(Path(hardware_config))["car_mode"]
    nav_share = get_package_share_directory("wheeltec_nav2")
    default_param_file = os.path.join(
        nav_share, "param", "wheeltec_params", f"param_{car_mode}.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mapper",
                default_value="gmapping",
                description="Mapping backend: gmapping or cartographer",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument(
                "start_safety",
                default_value="true",
                description="Start velocity smoothing and collision monitor",
            ),
            DeclareLaunchArgument("lidar_type", default_value=""),
            DeclareLaunchArgument(
                "lidar_type_yaml", default_value=hardware_config
            ),
            DeclareLaunchArgument(
                "params",
                default_value=default_param_file,
                description="Nav2 parameters used by the mapping safety gate",
            ),
            OpaqueFunction(function=include_mapper),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        nav_share, "launch", "mapping_safety.launch.py"
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params": param_file,
                }.items(),
                condition=IfCondition(start_safety),
            ),
        ]
    )
