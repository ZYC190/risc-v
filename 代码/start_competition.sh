#!/usr/bin/env bash
set -e
cd /home/zyc/robot2
source /opt/ros/humble/setup.bash
source install/setup.bash
exec ros2 launch home_patrol competition_system.launch.py
