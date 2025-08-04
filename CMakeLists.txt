
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
find_package(sensor_msgs REQUIRED) # Added sensor_msgs
find_package(tf2_ros REQUIRED) # Added tf2_ros
find_package(tf2 REQUIRED) # Added tf2

# --- Executable for the Simulation Node ---
# This builds the C++ code that simulates a differential drive robot.
add_executable(slampibot_gazebo_node
  src/sim_drive_node.cpp
)

ament_target_dependencies(slampibot_gazebo_node 
    rclcpp 
    geometry_msgs 
    nav_msgs
    sensor_msgs # Added sensor_msgs
    tf2_ros # Added tf2_ros
    tf2 # Added tf2
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
