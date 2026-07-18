from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Keep the official-style launch name, but route it through the same
    # publisher checks, backup, validation and atomic commit as save_map.sh.
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["/home/zyc/robot2/scripts/save_map.sh"],
                name="safe_map_saver",
                output="screen",
            )
        ]
    )
