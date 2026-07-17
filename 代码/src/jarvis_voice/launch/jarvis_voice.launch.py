from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="jarvis_voice",
            executable="air_sensor_node",
            name="air_sensor_node",
            output="screen",
        ),
        Node(
            package="jarvis_voice",
            executable="jarvis_node",
            name="jarvis_commander_node",
            output="screen",
        )
    ])
