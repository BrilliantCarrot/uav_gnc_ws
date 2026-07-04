cmake_minimum_required(VERSION 3.8)
project(uav_evaluation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(tracking_eval_node
  src/tracking_eval_node.cpp
)

add_executable(planning_path_logger_node
  src/planning_path_logger_node.cpp
)

ament_target_dependencies(tracking_eval_node
  rclcpp nav_msgs
)

ament_target_dependencies(planning_path_logger_node
  rclcpp nav_msgs
)

install(TARGETS
  tracking_eval_node
  planning_path_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
