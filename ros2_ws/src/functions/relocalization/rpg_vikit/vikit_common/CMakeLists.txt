CMAKE_MINIMUM_REQUIRED (VERSION 3.5)
SET(PROJECT_NAME vikit_common)
PROJECT(${PROJECT_NAME})
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas)
# Keep Eigen allocation compatible with system libraries on every architecture.
# No host-specific AVX/NEON or finite-math assumptions are exported.
# Add plain cmake packages 
FIND_PACKAGE(ament_cmake REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED core calib3d features2d highgui imgproc)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(fmt REQUIRED)

# Set Sourcefiles
LIST(APPEND SOURCEFILES src/atan_camera.cpp
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
                        src/img_align.cpp)

# Create vikit library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  opencv_core
  opencv_calib3d
  opencv_features2d
  opencv_highgui
  opencv_imgproc
  Sophus::Sophus
  fmt::fmt)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

option(VIKIT_COMMON_BUILD_TESTS "Build standalone Vikit development programs" OFF)
if(VIKIT_COMMON_BUILD_TESTS)
  foreach(test_name camera triangulation patch_score)
    add_executable(test_vk_common_${test_name} test/test_${test_name}.cpp)
    target_link_libraries(test_vk_common_${test_name} ${PROJECT_NAME})
  endforeach()
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  OpenCV
  Sophus
  fmt
)
ament_export_targets(${PROJECT_NAME})

ament_package()
