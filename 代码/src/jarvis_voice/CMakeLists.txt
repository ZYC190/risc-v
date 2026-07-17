cmake_minimum_required(VERSION 3.8)
project(jarvis_voice)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/air_sensor_node
  scripts/jarvis_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/air_sensor_node.py
  scripts/jarvis_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
