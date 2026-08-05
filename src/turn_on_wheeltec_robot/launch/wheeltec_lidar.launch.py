import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"Invalid lidar configuration: {path}")
    return config


def launch_value(value) -> str:
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def include_lidar_launch(context, *args, **kwargs):
    del args, kwargs
    config_path = Path(LaunchConfiguration("lidar_type_yaml").perform(context))
    config = load_yaml(config_path)
    lidar_type = LaunchConfiguration("lidar_type").perform(context).strip()
    if not lidar_type:
        lidar_type = str(config.get("lidar_type", "")).strip()
    if not lidar_type:
        raise ValueError(f"lidar_type is missing from {config_path}")

    if lidar_type.startswith("ls"):
        if lidar_type == "lscx":
            template_yaml = Path(
                get_package_share_directory("lslidar_driver"),
                "config",
                "lslidar_cx.yaml",
            )
            cx_config = load_yaml(template_yaml)["cx"]["lslidar_driver_node"][
                "ros__parameters"
            ]
            crop = config.get("lscx", {})
            if crop.get("angle_disable_min") != 0 and crop.get(
                "angle_disable_max"
            ) != 0:
                cx_config["angle_disable_min"] = crop["angle_disable_min"]
                cx_config["angle_disable_max"] = crop["angle_disable_max"]

            lidar_action = GroupAction(
                actions=[
                    LifecycleNode(
                        package="lslidar_driver",
                        executable="lslidar_driver_node",
                        name="lslidar_driver_node",
                        namespace="cx",
                        parameters=[cx_config],
                        output="screen",
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory(
                                    "pointcloud_to_laserscan"
                                ),
                                "launch",
                                "pointcloud_to_laserscan_launch.py",
                            )
                        )
                    ),
                ]
            )
        else:
            template_yaml = Path(
                get_package_share_directory("lslidar_driver"),
                "config",
                "lslidar_x10.yaml",
            )
            x10_config = load_yaml(template_yaml)["x10"]["lslidar_driver_node"][
                "ros__parameters"
            ]
            x10_user = config.get("x10", {})
            if lidar_type.endswith("net"):
                x10_config["serial_port"] = ""
            elif lidar_type.endswith("uart"):
                x10_config["serial_port"] = x10_user["lidar_port"]

            if lidar_type.startswith("ls_M10"):
                x10_config["lidar_model"] = (
                    "M10P" if lidar_type.startswith("ls_M10P") else "M10"
                )
            elif lidar_type.startswith("ls_N10"):
                x10_config["lidar_model"] = (
                    "N10Plus"
                    if lidar_type.startswith("ls_N10Plus")
                    else "N10"
                )

            if x10_user.get("angle_disable_min") != 0 and x10_user.get(
                "angle_disable_max"
            ) != 0:
                x10_config["angle_disable_min"] = x10_user[
                    "angle_disable_min"
                ]
                x10_config["angle_disable_max"] = x10_user[
                    "angle_disable_max"
                ]

            lidar_action = LifecycleNode(
                package="lslidar_driver",
                executable="lslidar_driver_node",
                name="lslidar_driver_node",
                namespace="x10",
                parameters=[x10_config],
                output="screen",
            )
    elif lidar_type in {"ldstl19p", "ldstl06nbj", "ldstl19n"}:
        launch_files = {
            "ldstl19p": "stl19p.launch.py",
            "ldstl06nbj": "stl06nbj.launch.py",
            "ldstl19n": "stl19n.launch.py",
        }
        lidar_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("ldlidar"),
                    "launch",
                    launch_files[lidar_type],
                )
            )
        )
    elif lidar_type == "rplidar_c1":
        # This robot uses a Slamtec RPLIDAR C1 through a CP2102N adapter.  Keep
        # its serial, frame and crop settings in wheeltec_param.yaml so mapping
        # and navigation always consume the same /scan contract.
        rplidar_config = {
            "serial_port": "/dev/wheeltec_lidar",
            "serial_baudrate": 460800,
            "frame_id": "laser_link",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Standard",
            "enable_angle_crop_func": True,
            "angle_crop_min": 90.0,
            "angle_crop_max": 270.0,
        }
        rplidar_config.update(config.get("rplidar_c1", {}))
        launch_arguments = {
            key: launch_value(rplidar_config[key])
            for key in (
                "serial_port",
                "serial_baudrate",
                "frame_id",
                "inverted",
                "angle_compensate",
                "scan_mode",
                "enable_angle_crop_func",
                "angle_crop_min",
                "angle_crop_max",
            )
        }
        lidar_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("rplidar_ros"),
                    "launch",
                    "rplidar_c1_launch.py",
                )
            ),
            launch_arguments=launch_arguments.items(),
        )
    else:
        raise ValueError(f"Unsupported lidar_type: {lidar_type}")

    return [lidar_action]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("turn_on_wheeltec_robot"),
        "config",
        "wheeltec_param.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_type_yaml",
                default_value=default_config,
                description="Robot and lidar hardware configuration YAML",
            ),
            DeclareLaunchArgument(
                "lidar_type",
                default_value="",
                description="Override the lidar model from the hardware YAML",
            ),
            OpaqueFunction(function=include_lidar_launch),
        ]
    )
