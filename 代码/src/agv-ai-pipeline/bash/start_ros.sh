#!/bin/bash
set -e

# 加载 ROS2 环境
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "ROS2 setup.bash not found!"
  exit 1
fi

# 加载工作空间环境
if [ -f ~/jobot_ws/install/setup.bash ]; then
  source ~/jobot_ws/install/setup.bash
else
  echo "Workspace setup.bash not found!"
  exit 1
fi

# 确保 ros2 命令可用
if ! command -v ros2 &> /dev/null; then
  echo "ros2 command not found! Is ROS2 correctly installed?"
  exit 1
fi

# 启动 odometry 节点
echo "Launching myagv_only_move.launch.py..."
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot_mic.launch.py
# 启动 mic 节点
# echo "Running myagv_mic_node..."
# if ros2 run jobot_mic myagv_mic_node; then
#   echo "myagv_mic_node started successfully!"
# else
#   echo "Failed to start myagv_mic_node."
#   exit 1
# fi
