cmake_minimum_required(VERSION 3.8)
project(scenarios)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# launch 파일 설치
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()