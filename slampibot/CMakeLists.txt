
cmake_minimum_required(VERSION 3.8)
project(slampibot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies for the real robot
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) # Still needed for launch files and general ROS2 environment
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
# DynamixelSDK, controller_manager, hardware_interface, diff_drive_controller, pluginlib are no longer needed for building custom C++ nodes for the real robot

# --- No custom C++ executable for the real robot in this approach ---
# The OpenCR board runs the firmware, and rosserial_python bridges the communication.
# If you need to build any custom C++ nodes for the real robot, add them here.

# Install launch files, URDF, RViz configs etc. for the real robot
install(DIRECTORY 
    launch 
    myCar 
    rviz 
    config 
    parameter 
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
