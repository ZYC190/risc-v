import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_base = LaunchConfiguration("start_base")
    start_lidar = LaunchConfiguration("start_lidar")
    lidar_type = LaunchConfiguration("lidar_type")
    lidar_type_yaml = LaunchConfiguration("lidar_type_yaml")

    robot_share = get_package_share_directory("turn_on_wheeltec_robot")
    robot_launch_dir = os.path.join(robot_share, "launch")
    default_hardware_config = os.path.join(
        robot_share, "config", "wheeltec_param.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time",
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
                "lidar_type",
                default_value="",
                description="Override lidar model from the hardware YAML",
            ),
            DeclareLaunchArgument(
                "lidar_type_yaml",
                default_value=default_hardware_config,
                description="Robot and lidar hardware configuration YAML",
            ),
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
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
        ]
    )
