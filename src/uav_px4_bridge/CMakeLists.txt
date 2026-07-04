cmake_minimum_required(VERSION 3.8)
project(uav_px4_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs QUIET)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(px4_msgs_FOUND)
  add_executable(px4_bridge_node
    src/px4_bridge_node.cpp
  )

  add_executable(px4_odom_converter
    src/px4_odom_converter.cpp
  )

  ament_target_dependencies(px4_bridge_node
    rclcpp nav_msgs geometry_msgs px4_msgs tf2 tf2_geometry_msgs
  )

  ament_target_dependencies(px4_odom_converter
    rclcpp nav_msgs px4_msgs tf2
  )

  install(TARGETS
    px4_bridge_node
    px4_odom_converter
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "px4_msgs not found. Skipping px4_bridge_node and px4_odom_converter builds.")
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
