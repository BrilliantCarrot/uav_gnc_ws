cmake_minimum_required(VERSION 3.8)
project(uav_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(path_viz_node
  src/path_viz_node.cpp
)

ament_target_dependencies(path_viz_node
  rclcpp geometry_msgs nav_msgs sensor_msgs visualization_msgs
)

install(TARGETS
  path_viz_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
