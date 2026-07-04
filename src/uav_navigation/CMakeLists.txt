cmake_minimum_required(VERSION 3.8)
project(uav_navigation)

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
find_package(Eigen3 REQUIRED)

add_library(uav_navigation_core
  src/ekf.cpp
  src/ukf.cpp
)

target_include_directories(uav_navigation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(uav_navigation_core
  Eigen3
)

add_executable(navigation_node
  src/navigation_node.cpp
)

target_include_directories(navigation_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(navigation_node
  uav_navigation_core
)

ament_target_dependencies(navigation_node
  rclcpp geometry_msgs nav_msgs sensor_msgs Eigen3
)

install(TARGETS
  uav_navigation_core
  EXPORT export_uav_navigation_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(uav_navigation_core)
ament_export_targets(export_uav_navigation_core HAS_LIBRARY_TARGET)

ament_package()
