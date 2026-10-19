cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME vikit_common)
project(${PROJECT_NAME} LANGUAGES C CXX)

# ============================================================
# Basic configuration
# ============================================================

# Default to Release unless caller explicitly specifies otherwise
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_VERBOSE_MAKEFILE OFF)

list(APPEND CMAKE_MODULE_PATH
  "${PROJECT_SOURCE_DIR}/CMakeModules"
)

# Set TRUE only when building this package as ROS1/catkin.
# physic_bev / ROS2 Humble keeps this FALSE.
set(USE_ROS FALSE)

# ============================================================
# C++ standard
# ============================================================

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================
# Common compiler options
# ============================================================

add_compile_definitions(
  _LINUX
  _REENTRANT
)

add_compile_options(
  -Wall
  -Wno-unused-variable
  -Wno-unused-but-set-variable
  -Wno-unknown-pragmas
)

# ============================================================
# CPU architecture detection
#
# Supported:
#   - x86_64 Linux PC
#   - ARM64 / AArch64
#       RDK
#       RK3588
#       Jetson
#       other ARM64 Linux boards
#   - ARM32 legacy boards
# ============================================================

message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

string(
  TOLOWER
  "${CMAKE_SYSTEM_PROCESSOR}"
  SYSTEM_PROCESSOR_LOWER
)

if(SYSTEM_PROCESSOR_LOWER MATCHES "^(x86_64|amd64)$")

  message(STATUS "Building vikit_common for x86_64 Linux")

  # x86/x86_64 optimization flags
  add_compile_options(
    -march=native
    -mmmx
    -msse
    -msse2
    -msse3
    -mssse3
  )

elseif(SYSTEM_PROCESSOR_LOWER MATCHES "^(aarch64|arm64)$")

  message(STATUS "Building vikit_common for ARM64/AArch64 Linux")

  # RDK / RK3588 / Jetson are AArch64.
  #
  # DO NOT add x86-only options:
  #   -mmmx
  #   -msse
  #   -msse2
  #   -msse3
  #   -mssse3
  #
  # DO NOT add -mfpu=neon on AArch64.
  # Advanced SIMD / NEON is part of AArch64.
  #
  # Use conservative ARMv8-A target for broad ARM64 compatibility.
  add_compile_options(
    -march=armv8-a
  )

elseif(SYSTEM_PROCESSOR_LOWER MATCHES "^arm")

  message(STATUS "Building vikit_common for ARM32 Linux")

  # Legacy ARM32 boards
  add_compile_options(
    -mfpu=neon
  )

else()

  message(WARNING
    "Unknown CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}. "
    "Using generic compiler settings."
  )

endif()

# ============================================================
# Release optimization
# ============================================================

# Removed old -fsee flag.
# Keep optimizations portable between x86_64 and ARM64.
set(
  CMAKE_CXX_FLAGS_RELEASE
  "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops"
)

# ============================================================
# Dependencies
# ============================================================

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)

message(STATUS "OpenCV version: ${OpenCV_VERSION}")

if(DEFINED Eigen_VERSION)
  message(STATUS "Eigen version: ${Eigen_VERSION}")
endif()

# ============================================================
# Include directories
# ============================================================

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

# ============================================================
# Optional ROS1 / Catkin support
# ============================================================

if(USE_ROS)

  find_package(
    catkin REQUIRED
    COMPONENTS
      roscpp
      cmake_modules
  )

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

  catkin_package(
    DEPENDS
      Eigen
      OpenCV
      Sophus

    CATKIN_DEPENDS
      roscpp

    INCLUDE_DIRS
      include

    LIBRARIES
      ${PROJECT_NAME}
  )

else()

  # Preserve original vikit behavior.
  set(
    EXECUTABLE_OUTPUT_PATH
    ${PROJECT_SOURCE_DIR}/bin
  )

  set(
    LIBRARY_OUTPUT_PATH
    ${PROJECT_SOURCE_DIR}/lib
  )

endif()

# ============================================================
# Source files
# ============================================================

set(
  SOURCEFILES

  src/atan_camera.cpp
  src/omni_camera.cpp
  src/math_utils.cpp
  src/vision.cpp
  src/performance_monitor.cpp
  src/robust_cost.cpp
  src/user_input_thread.cpp
  src/pinhole_camera.cpp
  src/equidistant_camera.cpp
  src/polynomial_camera.cpp
  src/homography.cpp
  src/img_align.cpp
)

# ============================================================
# vikit_common shared library
# ============================================================

add_library(
  ${PROJECT_NAME}
  SHARED
  ${SOURCEFILES}
)

target_include_directories(
  ${PROJECT_NAME}

  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ${PROJECT_NAME}

  PUBLIC
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    fmt::fmt
)

if(USE_ROS)

  target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
      ${catkin_LIBRARIES}
  )

endif()

# ============================================================
# Tests
# ============================================================

option(VIKIT_COMMON_BUILD_TESTS "Build standalone vikit_common development tests" OFF)

if(VIKIT_COMMON_BUILD_TESTS)
  add_executable(
    test_vk_common_camera
    test/test_camera.cpp
  )

  target_link_libraries(
    test_vk_common_camera
    PRIVATE
      ${PROJECT_NAME}
      ${OpenCV_LIBS}
  )

  add_executable(
    test_vk_common_triangulation
    test/test_triangulation.cpp
  )

  target_link_libraries(
    test_vk_common_triangulation
    PRIVATE
      ${PROJECT_NAME}
      ${OpenCV_LIBS}
  )

  add_executable(
    test_vk_common_patch_score
    test/test_patch_score.cpp
  )

  target_link_libraries(
    test_vk_common_patch_score
    PRIVATE
      ${PROJECT_NAME}
      ${OpenCV_LIBS}
  )
endif()

# ============================================================
# Plain-CMake package export
#
# IMPORTANT:
#
# Do NOT export this:
#
#   $<TARGET_FILE:vikit_common>
#
# into vikit_commonConfig.cmake.
#
# That generator expression only works inside the CMake project
# which actually defines target "vikit_common".
#
# fast_livo is another CMake project, therefore it cannot resolve:
#
#   $<TARGET_FILE:vikit_common>
#
# Here we export a real installed library path instead.
# ============================================================

if(NOT USE_ROS)

  # ----------------------------------------------------------
  # Installation paths
  # ----------------------------------------------------------

  set(
    vikit_common_INCLUDE_DIR
    "${CMAKE_INSTALL_PREFIX}/include"
  )

  set(
    vikit_common_INCLUDE_DIRS
    "${CMAKE_INSTALL_PREFIX}/include"
  )

  set(
    vikit_common_LIBRARY_DIR
    "${CMAKE_INSTALL_PREFIX}/lib"
  )

  set(
    vikit_common_LIBRARY_DIRS
    "${CMAKE_INSTALL_PREFIX}/lib"
  )

  # Actual installed shared-library filename:
  #
  # Linux:
  #   libvikit_common.so
  #
  # This is deliberately a concrete path instead of
  # $<TARGET_FILE:vikit_common>.
  set(
    vikit_common_LIBRARIES
    "${CMAKE_INSTALL_PREFIX}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX}"
  )

  set(
    VIKIT_COMMON_LOCATION
    "${vikit_common_LIBRARIES}"
  )

  message(STATUS "vikit_common install prefix : ${CMAKE_INSTALL_PREFIX}")
  message(STATUS "vikit_common include dir    : ${vikit_common_INCLUDE_DIRS}")
  message(STATUS "vikit_common library dir    : ${vikit_common_LIBRARY_DIRS}")
  message(STATUS "vikit_common library        : ${vikit_common_LIBRARIES}")

  # ----------------------------------------------------------
  # Generate vikit_commonConfig.cmake
  # ----------------------------------------------------------

  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/vikit_commonConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake
    @ONLY
  )

  # Preserve compatibility with projects using CMake user
  # package registry.
  export(PACKAGE vikit_common)

  # ----------------------------------------------------------
  # Install headers
  # ----------------------------------------------------------

  install(
    DIRECTORY
      ${PROJECT_SOURCE_DIR}/include/vikit

    DESTINATION
      include

    FILES_MATCHING
      PATTERN "*.h"
      PATTERN "*.hpp"
  )
  

  # ----------------------------------------------------------
  # Install shared library
  # ----------------------------------------------------------

  install(
    TARGETS
      ${PROJECT_NAME}

    LIBRARY DESTINATION
      lib

    ARCHIVE DESTINATION
      lib

    RUNTIME DESTINATION
      bin
  )

  # ----------------------------------------------------------
  # Install CMake package config
  #
  # This makes:
  #
  #   find_package(vikit_common REQUIRED)
  #
  # work from downstream packages after colcon installation.
  # ----------------------------------------------------------

  install(
    FILES
      ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake

    DESTINATION
      share/vikit_common/cmake
  )

endif()
