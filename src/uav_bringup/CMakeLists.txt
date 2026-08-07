cmake_minimum_required(VERSION 3.8)
project(uav_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)

add_executable(sim_odom_to_gazebo_pose_node
  src/sim_odom_to_gazebo_pose_node.cpp
)

add_executable(lio_validation_trajectory_node
  src/lio_validation_trajectory_node.cpp
)

add_executable(lio_validation_error_node
  src/lio_validation_error_node.cpp
)

ament_target_dependencies(sim_odom_to_gazebo_pose_node
  rclcpp
  nav_msgs
)

ament_target_dependencies(lio_validation_trajectory_node
  rclcpp
  nav_msgs
  sensor_msgs
)

ament_target_dependencies(lio_validation_error_node
  rclcpp
  nav_msgs
)

target_link_libraries(sim_odom_to_gazebo_pose_node
  ignition-msgs8::ignition-msgs8
  ignition-transport11::ignition-transport11
)

install(TARGETS
  sim_odom_to_gazebo_pose_node
  lio_validation_trajectory_node
  lio_validation_error_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
