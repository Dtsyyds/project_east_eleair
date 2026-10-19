cmake_minimum_required(VERSION 3.8)
project(eddy_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EddyFrame.msg"
  "msg/EddyChannelMean.msg"
  "msg/EddyStatus.msg"
  "msg/EddyCommand.msg"
  DEPENDENCIES std_msgs
)

add_library(eddy_driver_core SHARED
  src/eddy_driver.cpp
)

target_include_directories(eddy_driver_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(eddy_driver_core
  Threads::Threads
)

add_executable(eddy_driver_node
  src/eddy_node.cpp
  src/eddy_main.cpp
)

target_include_directories(eddy_driver_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(eddy_driver_node
  eddy_driver_core
  Threads::Threads
)

ament_target_dependencies(eddy_driver_node
  rclcpp
  std_msgs
)

# 将本功能包生成的 C++ 消息类型支持链接到节点。
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(eddy_driver_node "${cpp_typesupport_target}")

install(
  TARGETS
    eddy_driver_core
    eddy_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config udev
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(eddy_driver_core)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
