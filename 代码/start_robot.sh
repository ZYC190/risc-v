#!/bin/bash
echo "🚀 正在初始化小微 AMR 核心系统..."

# 1. 加载 ROS2 和你自己的环境变量（极其重要，不加绝对起不来）
source /opt/ros/humble/setup.bash
source /home/zyc/robot2/install/setup.bash

# 2. 一键启动你刚刚做好的全家桶！
ros2 launch my_robot_bringup start_all.launch.py