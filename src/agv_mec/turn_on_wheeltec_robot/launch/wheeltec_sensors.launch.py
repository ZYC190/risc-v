import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # 定义命令行参数
    enable_robot = LaunchConfiguration('enable_robot', default='true')
    enable_lidar = LaunchConfiguration('enable_lidar', default='true')
    enable_camera = LaunchConfiguration('enable_camera', default='true')

    # 声明命令行参数
    robot_arg = DeclareLaunchArgument(
        'enable_robot', default_value='true',
        description='Enable the wheeltec robot launch file'
    )
    lidar_arg = DeclareLaunchArgument(
        'enable_lidar', default_value='true',
        description='Enable the wheeltec lidar launch file'
    )
    camera_arg = DeclareLaunchArgument(
        'enable_camera', default_value='true',
        description='Enable the wheeltec camera launch file'
    )

    # 根据参数条件加载不同的 launch 文件
    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
        condition=IfCondition(enable_robot),
    )
    lidar_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_lidar.launch.py')),
        condition=IfCondition(enable_lidar),
    )
    wheeltec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_camera.launch.py')),
        condition=IfCondition(enable_camera),
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        # 声明参数
        robot_arg,
        lidar_arg,
        camera_arg,

        # 加载条件性的 launch 文件
        wheeltec_robot,
        lidar_ros,
        wheeltec_camera,
    ])
