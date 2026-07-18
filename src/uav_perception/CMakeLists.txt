cmake_minimum_required(VERSION 3.8)
project(uav_perception)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(uav_dynamics REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_executable(virtual_lidar_node
  src/virtual_lidar_node.cpp
)

add_executable(raw_lidar_sim_node
  src/raw_lidar_sim_node.cpp
)

add_executable(imu_lio_adapter_node
  src/imu_lio_adapter_node.cpp
)

add_executable(lidar_preprocess_node
  src/lidar_preprocess_node.cpp
)

add_executable(lidar_pose_correction_node
  src/lidar_pose_correction_node.cpp
)

add_executable(occupancy_projection_node
  src/occupancy_projection_node.cpp
)

ament_target_dependencies(virtual_lidar_node
  rclcpp nav_msgs sensor_msgs uav_dynamics
)

ament_target_dependencies(raw_lidar_sim_node
  rclcpp nav_msgs sensor_msgs uav_dynamics
)

ament_target_dependencies(imu_lio_adapter_node
  rclcpp sensor_msgs
)

ament_target_dependencies(lidar_preprocess_node
  rclcpp sensor_msgs pcl_conversions
)

target_link_libraries(lidar_preprocess_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(lidar_pose_correction_node
  rclcpp nav_msgs sensor_msgs
)

ament_target_dependencies(occupancy_projection_node
  rclcpp sensor_msgs nav_msgs visualization_msgs pcl_conversions
)

target_link_libraries(occupancy_projection_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  virtual_lidar_node
  raw_lidar_sim_node
  imu_lio_adapter_node
  lidar_preprocess_node
  lidar_pose_correction_node
  occupancy_projection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
