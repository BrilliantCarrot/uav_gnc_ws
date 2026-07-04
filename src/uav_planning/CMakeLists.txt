cmake_minimum_required(VERSION 3.8)
project(uav_planning)

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
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(uav_planning_core
  src/dstar_lite.cpp
)

target_include_directories(uav_planning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(uav_planning_core
  Eigen3
)

add_executable(path_planner_node
  src/path_planner_node.cpp
)

target_include_directories(path_planner_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(path_planner_node
  uav_planning_core
)

ament_target_dependencies(path_planner_node
  rclcpp nav_msgs geometry_msgs visualization_msgs Eigen3
)

install(TARGETS
  uav_planning_core
  EXPORT export_uav_planning_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(uav_planning_core)
ament_export_targets(export_uav_planning_core HAS_LIBRARY_TARGET)

ament_package()
