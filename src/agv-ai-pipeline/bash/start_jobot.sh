#!/bin/bash
set -e  # 如果有错误，脚本立刻退出

# 加载ROS2 Humble环境

# 激活Python虚拟环境
source ~/ai_env/bin/activate
source /opt/ros/humble/setup.bash


# 进入工作目录
cd ~/jobot-ai-pipeline

# 启动节点
echo "Starting agv_follow_node.py..."
python agv_follow_node.py &   # 后台启动

sleep 4  # 可选，给第一个节点留点时间

echo "Starting agv_master_node.py..."
python agv_master_node.py     # 前台启动，直到它退出

