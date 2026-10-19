cmake_minimum_required(VERSION 3.8)
project(tank_vision_core)

if(DEFINED PYTHON_EXECUTABLE)
  message(STATUS "Legacy PYTHON_EXECUTABLE requested: ${PYTHON_EXECUTABLE}")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(
  tank_vision
  PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../tank_vision"
)

ament_package()
