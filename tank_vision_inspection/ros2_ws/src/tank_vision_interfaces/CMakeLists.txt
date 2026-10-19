cmake_minimum_required(VERSION 3.8)
project(tank_vision_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Defect.msg"
  "msg/DefectArray.msg"
  "msg/SeamState.msg"
  "msg/SeamGeometry.msg"
  "msg/VisionQuality.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
