cmake_minimum_required(VERSION 3.8)
project(tank_vision_core)
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(INSPECTION_CHECKOUT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../.." ABSOLUTE)
# pip owns tank_vision. deploy/build.py installs the one canonical source first.
execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import pathlib, tank_vision; expected=pathlib.Path(r'${INSPECTION_CHECKOUT}/packages/tank_vision/tank_vision').resolve(); assert pathlib.Path(tank_vision.__file__).resolve().parent == expected, tank_vision.__file__; print(tank_vision.__file__)"
  RESULT_VARIABLE TANK_VISION_IMPORT_STATUS OUTPUT_VARIABLE TANK_VISION_ORIGIN ERROR_VARIABLE TANK_VISION_ERROR)
if(NOT TANK_VISION_IMPORT_STATUS EQUAL 0)
  message(FATAL_ERROR "Install packages/tank_vision with deploy/build.sh first: ${TANK_VISION_ERROR}")
endif()
message(STATUS "pip-owned tank_vision: ${TANK_VISION_ORIGIN}")
ament_package()
