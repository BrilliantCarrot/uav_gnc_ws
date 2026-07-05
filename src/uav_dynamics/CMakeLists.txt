cmake_minimum_required(VERSION 3.8)
project(uav_dynamics)

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

add_library(uav_dynamics_core
  src/sixdof.cpp
  src/multirotor_model.cpp
)

target_include_directories(uav_dynamics_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(simulation_node
  src/simulation_node.cpp
)

target_link_libraries(simulation_node
  uav_dynamics_core
)

ament_target_dependencies(simulation_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
)

install(TARGETS
  uav_dynamics_core
  EXPORT export_uav_dynamics_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  simulation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(uav_dynamics_core)
ament_export_targets(export_uav_dynamics_core HAS_LIBRARY_TARGET)

ament_package()
