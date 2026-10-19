cmake_minimum_required(VERSION 3.8)
project(agv_bridge)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(agv_protocol REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenSSL REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(eddy_driver REQUIRED)
find_package(inspection_interfaces REQUIRED)

# 添加包含目录
include_directories(include)

# 创建库
add_library(${PROJECT_NAME} SHARED
  src/bridge_manager.cpp
  src/tank_inspection_robot.cpp
)

# 链接依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  eddy_driver
  inspection_interfaces
  agv_protocol
)

target_link_libraries(${PROJECT_NAME} agv_protocol::agv_protocol ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

# 创建独立 WebSocket 桥接节点
add_executable(websocket_bridge_node src/websocket_bridge_node.cpp)
target_link_libraries(websocket_bridge_node 
  ${PROJECT_NAME} 
  agv_protocol::agv_protocol
  Boost::system
  Boost::thread
  OpenSSL::SSL
  OpenSSL::Crypto
)
ament_target_dependencies(websocket_bridge_node rclcpp geometry_msgs std_msgs nav_msgs sensor_msgs)

# 导出头文件
install(DIRECTORY include/ DESTINATION include)

# 导出库
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

# 安装可执行文件（不导出）
install(TARGETS websocket_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# 导出 cmake 配置以便其他包使用
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp 
  geometry_msgs 
  std_msgs 
  nav_msgs 
  sensor_msgs 
  agv_protocol
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(inspection_bridge_e2e test/test_inspection_bridge.py TIMEOUT 90)
endif()
ament_package()
