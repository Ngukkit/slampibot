cmake_minimum_required(VERSION 3.8)
project(robot_commander)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  robot_commander/waypoint_commander_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()