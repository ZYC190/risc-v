cmake_minimum_required(VERSION 3.8)
project(robot_mqtt_bridge)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/unified_bridge
  scripts/unified_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
