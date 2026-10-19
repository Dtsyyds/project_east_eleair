cmake_minimum_required(VERSION 3.8)
project(inspection_mock_sensors)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(inspection_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(mock_sensors_node src/mock_sensors_node.cpp)
target_compile_features(mock_sensors_node PUBLIC cxx_std_17)
ament_target_dependencies(mock_sensors_node
  geometry_msgs inspection_interfaces nav_msgs rclcpp sensor_msgs tf2_ros)

install(TARGETS mock_sensors_node DESTINATION lib/${PROJECT_NAME})
ament_package()
