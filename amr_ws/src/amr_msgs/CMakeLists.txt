cmake_minimum_required(VERSION 3.8)
project(amr_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# msg 파일 등록
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlLatency.msg"
  "msg/PoseRmse.msg"
  "msg/MinObstacleDistance.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()