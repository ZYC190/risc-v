#!/bin/bash
set -e  # 如果有错误，脚本立刻退出

sleep 20
# 设置 ROS 相关环境变量
export ROS_DOMAIN_ID=77

# 激活Python虚拟环境
source /opt/ros/humble/setup.bash
source /home/er/ai_env/bin/activate


# 进入工作目录
cd /home/er/jobot-ai-pipeline

echo "Starting agv_master_with_nav.py..."
python agv_master_with_nav.py     # 前台启动，直到它退出

