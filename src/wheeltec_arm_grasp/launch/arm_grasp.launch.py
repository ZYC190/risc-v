#!/usr/bin/env python3
"""
启动文件: WHEELTEC 六轴机械臂双目视觉抓取

启动内容:
  1. static_transform_publisher: 相机 → 机械臂基座坐标变换 (眼在手外)
  2. wheeltec_arm_grasp_node: 主抓取控制节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    """生成 launch 描述"""
    pkg_share = get_package_share_directory('wheeltec_arm_grasp')
    params_file = os.path.join(pkg_share, 'config', 'arm_params.yaml')

    # 相机 → 机械臂 静态 TF 参数
    # ⚠️ 这些值需要根据实际安装位置标定!
    # 可以通过命令行覆盖: ros2 launch ... x:=0.35 y:=-0.12 z:=0.42
    tf_x = LaunchConfiguration('tf_x', default='0.0')
    tf_y = LaunchConfiguration('tf_y', default='-0.20')
    tf_z = LaunchConfiguration('tf_z', default='0.25')
    tf_roll = LaunchConfiguration('tf_roll', default='1.57')
    tf_pitch = LaunchConfiguration('tf_pitch', default='0.0')
    tf_yaw = LaunchConfiguration('tf_yaw', default='3.14')

    return LaunchDescription([
        # ==========================================
        # 可覆盖的启动参数
        # ==========================================
        DeclareLaunchArgument(
            'tf_x',
            default_value='0.0',
            description='相机在臂座 X 方向的偏移 (米, 左右)'
        ),
        DeclareLaunchArgument(
            'tf_y',
            default_value='-0.20',
            description='相机在臂座 Y 方向的偏移 (米, 前后, 负值=后方20cm)'
        ),
        DeclareLaunchArgument(
            'tf_z',
            default_value='0.25',
            description='相机在臂座 Z 方向的偏移 (米, 高度25cm)'
        ),
        DeclareLaunchArgument(
            'tf_roll',
            default_value='1.57',
            description='相机绕 X 轴旋转 (弧度, 光轴从"上"转"前")'
        ),
        DeclareLaunchArgument(
            'tf_pitch',
            default_value='0.0',
            description='相机绕 Y 轴旋转 (弧度, 负值=低头看近处)'
        ),
        DeclareLaunchArgument(
            'tf_yaw',
            default_value='3.14',
            description='相机绕 Z 轴旋转 (弧度, 3.14=光轴朝臂座前方)'
        ),

        # ==========================================
        # 静态 TF: 相机 → 机械臂基座 (眼在手外)
        # ==========================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_arm_base_tf',
            output='screen',
            arguments=[
                '--x', tf_x,
                '--y', tf_y,
                '--z', tf_z,
                '--roll', tf_roll,
                '--pitch', tf_pitch,
                '--yaw', tf_yaw,
                '--frame-id', 'table_arm_base_link',
                '--child-frame-id', 'camera_color_optical_frame',
            ],
        ),

        # ==========================================
        # 主抓取节点
        # ==========================================
        Node(
            package='wheeltec_arm_grasp',
            executable='arm_grasp_node',
            name='wheeltec_arm_grasp_node',
            parameters=[params_file],
            output='screen',
            emulate_tty=True,
        ),
    ])
