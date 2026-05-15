from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("table_streeing_arm", package_name="arm_moveit_config").to_moveit_configs()
    moveit_config.move_group_capabilities["capabilities"] = ""
    moveit_config.move_group_capabilities["disable_capabilities"] = ""
    return generate_demo_launch(moveit_config)
