cmake_minimum_required(VERSION 3.8)
project(slampibot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies for simulation
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(gazebo_ros_pkgs REQUIRED) # Often needed for Gazebo plugins

# --- Executable for the Simulation Node ---
# This node might be a dummy driver or interface for the simulation.
add_executable(slampibot_gazebo_node
  src/sim_drive_node.cpp
  # Add other source files needed for the simulation node here
)

ament_target_dependencies(slampibot_gazebo_node 
    rclcpp 
    geometry_msgs 
    nav_msgs
)

install(TARGETS
  slampibot_gazebo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install all directories needed for the simulation
install(DIRECTORY 
    launch 
    maps 
    params 
    rviz 
    myCar 
    worlds 
    parameter 
    config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()