cmake_minimum_required(VERSION 3.8)
project(uav_gnc)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# C++ 표준 지정(C++17로)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성, find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(px4_msgs REQUIRED)          # PX4 Offboard 브리지용

# LiDAR / PointCloud / PCL용 추가
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# core sources 들
set(CORE_SOURCES
  src/dynamics/sixdof.cpp
  src/control/controller.cpp
  src/trajectory.cpp
)

# core 라이브러리 타겟 생성
add_library(uav_gnc_core ${CORE_SOURCES})
add_library(ekf_lib src/ekf.cpp)
add_library(ukf_lib src/ukf.cpp)                 # UKF 라이브러리
add_library(dstar_lib src/planning/dstar_lite.cpp)  # D* Lite 라이브러리

# core가 참조할 헤더 경로(패키지 include)
target_include_directories(uav_gnc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(ekf_lib PUBLIC include)
target_include_directories(ukf_lib PUBLIC include)
target_include_directories(dstar_lib PUBLIC include)

# core 라이브러리 의존성
ament_target_dependencies(uav_gnc_core
  rclcpp geometry_msgs nav_msgs sensor_msgs Eigen3
)

ament_target_dependencies(ekf_lib
  rclcpp Eigen3
)

ament_target_dependencies(ukf_lib
  rclcpp Eigen3
)

ament_target_dependencies(dstar_lib
  rclcpp Eigen3
)

# 노드 실행파일 추가
add_executable(simulation_node src/simulation_node.cpp)
add_executable(control_node src/control_node.cpp)
add_executable(guidance_node src/guidance_node.cpp)
add_executable(navigation_node src/navigation_node.cpp)
add_executable(path_viz_node src/visualization/path_viz_node.cpp)
add_executable(tracking_eval_node src/evaluation/tracking_eval_node.cpp)
add_executable(px4_bridge_node src/px4_bridge_node.cpp)
add_executable(px4_odom_converter src/px4_odom_converter.cpp)
add_executable(path_planner_node src/planning/path_planner_node.cpp)
add_executable(virtual_lidar_node src/perception/virtual_lidar_node.cpp)
add_executable(lidar_preprocess_node src/perception/lidar_preprocess_node.cpp)
add_executable(lidar_pose_correction_node src/perception/lidar_pose_correction_node.cpp)
add_executable(occupancy_projection_node src/perception/occupancy_projection_node.cpp)
add_executable(planning_path_logger_node src/evaluation/planning_path_logger_node.cpp)

# 노드 링크
target_link_libraries(simulation_node uav_gnc_core)
target_link_libraries(control_node uav_gnc_core)
target_link_libraries(guidance_node uav_gnc_core)
target_link_libraries(navigation_node uav_gnc_core ekf_lib ukf_lib)
target_link_libraries(path_planner_node dstar_lib)

# PCL 링크
target_link_libraries(lidar_preprocess_node ${PCL_LIBRARIES})
target_link_libraries(occupancy_projection_node ${PCL_LIBRARIES})

# 노드 타겟 의존성 연결
ament_target_dependencies(simulation_node
  rclcpp geometry_msgs nav_msgs sensor_msgs
)

ament_target_dependencies(control_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
)

ament_target_dependencies(guidance_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs
)

ament_target_dependencies(navigation_node
  rclcpp geometry_msgs nav_msgs sensor_msgs Eigen3
)

ament_target_dependencies(path_viz_node
  rclcpp geometry_msgs nav_msgs sensor_msgs visualization_msgs
)

ament_target_dependencies(tracking_eval_node
  rclcpp nav_msgs
)

ament_target_dependencies(px4_bridge_node
  rclcpp nav_msgs geometry_msgs px4_msgs tf2 tf2_geometry_msgs
)

ament_target_dependencies(px4_odom_converter
  rclcpp nav_msgs px4_msgs tf2
)

ament_target_dependencies(path_planner_node
  rclcpp nav_msgs geometry_msgs visualization_msgs Eigen3
)

ament_target_dependencies(lidar_preprocess_node
  rclcpp sensor_msgs pcl_conversions
)
ament_target_dependencies(lidar_pose_correction_node
  rclcpp nav_msgs sensor_msgs
)
ament_target_dependencies(occupancy_projection_node
  rclcpp sensor_msgs nav_msgs visualization_msgs pcl_conversions
)

ament_target_dependencies(virtual_lidar_node
  rclcpp nav_msgs sensor_msgs
)
ament_target_dependencies(planning_path_logger_node
  rclcpp nav_msgs)

# 설치(install) 규칙
install(TARGETS
  uav_gnc_core
  ekf_lib
  ukf_lib
  dstar_lib
  simulation_node
  control_node
  guidance_node
  navigation_node
  path_viz_node
  tracking_eval_node
  px4_bridge_node
  px4_odom_converter
  path_planner_node
  lidar_preprocess_node
  lidar_pose_correction_node
  occupancy_projection_node
  virtual_lidar_node
  planning_path_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

# 설치: 헤더
install(DIRECTORY include/
  DESTINATION include
)

# 설치: launch/config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()