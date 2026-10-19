cmake_minimum_required(VERSION 3.12)
project(agv_protocol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 基础依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(jsoncpp REQUIRED)

# 查找 Boost（websocketpp 需要）
find_package(Boost REQUIRED COMPONENTS system thread)

# ============ WebSocketPP 配置 ============
find_path(WEBSOCKETPP_INCLUDE_DIR
  NAMES websocketpp/client.hpp
  PATHS
    /usr/include
    /usr/local/include
    /opt/local/include
)

if(NOT WEBSOCKETPP_INCLUDE_DIR AND DEFINED ENV{TANK_OFFLINE_PREFIX})
  find_path(WEBSOCKETPP_INCLUDE_DIR websocketpp/client.hpp
    PATHS "$ENV{TANK_OFFLINE_PREFIX}/usr/include" NO_DEFAULT_PATH)
endif()
if(NOT WEBSOCKETPP_INCLUDE_DIR)
  message(FATAL_ERROR "websocketpp missing; extract offline/debs/libwebsocketpp-dev*.deb and source tank_inspection_system/scripts/env.sh. Network fetch is disabled.")
endif()

message(STATUS "websocketpp include dir: ${WEBSOCKETPP_INCLUDE_DIR}")

# ============ ASIO 配置 ============
# 使用 Boost.ASIO（系统已安装 Boost）
find_package(Boost REQUIRED COMPONENTS system)

# websocketpp 默认使用 Boost.ASIO
message(STATUS "Using Boost.ASIO from: ${Boost_INCLUDE_DIRS}")
set(ASIO_INCLUDE_DIR ${Boost_INCLUDE_DIRS})

# 定义库
add_library(
  ${PROJECT_NAME}
  SHARED
  src/command_parser.cpp
  src/websocket_client.cpp
  src/websocket_server.cpp
)

# 创建别名 target 以便其他包可以链接
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# 添加编译定义（不使用 ASIO_STANDALONE，让 websocketpp 使用 Boost.ASIO）
target_compile_definitions(${PROJECT_NAME} PRIVATE
  # 不定义 ASIO_STANDALONE，使用 Boost.ASIO
)

# ROS2 依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  jsoncpp
)

# 普通 CMake target 依赖
target_link_libraries(${PROJECT_NAME}
  Boost::system
  Boost::thread
  jsoncpp_lib
  pthread
)

# 添加包含目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${WEBSOCKETPP_INCLUDE_DIR}
  ${ASIO_INCLUDE_DIR}
  /usr/include/jsoncpp
)

# 安装
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

# 导出配置
install(
  EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  jsoncpp
  Boost
)

ament_package()
