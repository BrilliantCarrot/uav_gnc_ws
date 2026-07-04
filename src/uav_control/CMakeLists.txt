cmake_minimum_required(VERSION 3.8)
project(uav_control)

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
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(uav_dynamics REQUIRED)

add_library(uav_control_core
  src/controller.cpp
)

target_include_directories(uav_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(uav_control_core
  uav_dynamics Eigen3
)

add_executable(control_node
  src/control_node.cpp
)

target_link_libraries(control_node
  uav_control_core
)

ament_target_dependencies(control_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs uav_dynamics
)

install(TARGETS
  uav_control_core
  EXPORT export_uav_control_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(uav_control_core)
ament_export_targets(export_uav_control_core HAS_LIBRARY_TARGET)
ament_export_dependencies(uav_dynamics)

ament_package()
