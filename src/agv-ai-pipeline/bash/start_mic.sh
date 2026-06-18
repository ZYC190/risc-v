#!/bin/bash
set -e

# 设置 ROS 相关环境变量
export ROS_DOMAIN_ID=77
# 加载 ROS2 环境
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
else
  echo "ROS2 setup.bash not found!"
  exit 1
fi

# 加载工作空间环境
if [ -f /home/er/br_ws/install/setup.bash ]; then
  source /home/er/br_ws/install/setup.bash
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
ros2 run jobot_mic myagv_mic_node

