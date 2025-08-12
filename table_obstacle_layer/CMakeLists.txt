cmake_minimum_required(VERSION 3.8)
project(table_obstacle_layer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(costmap_2d REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/polygon_layer.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  nav2_costmap_2d
  geometry_msgs
  costmap_2d
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugin/plugin.xml)

target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(FILES plugin/plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()
