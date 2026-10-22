# M0 迁移映射与 M1 准备

日期：2026-10-22；基准提交 `b34a3c29c62500915c302b340cb4d51880b00980`。以下是计划映射，本轮未执行移动。所有路径相对新工作目录；保留已有 ROS 包边界、package.xml 包名、导入名和消息类型。

## 目标目录职责

| 目标 | 拟承接现有内容 |
|---|---|
| apps/ | `system_websocket/` → `apps/system_websocket/`；`websocket_console/` → `apps/websocket_console/`。保留应用进程边界。 |
| packages/ | `tank_vision_inspection/` 的非 ROS Python 分发 → `packages/tank_vision_inspection/`（含 tank_vision/、pyproject.toml、setup.py、requirements/）；D405 分发 → `packages/d405_seam_tracking/`（含 src/、native/、pyproject.toml）。`physic_bev/src/emat/` 拟整体归入 `packages/emat/`，目前没有 ROS package.xml，后续再补 Python 安装边界并更新应用动态导入。下列 ROS 包单独迁出，不复制算法源码。 |
| ros2_ws/src/drivers/ | 相机、LiDAR、Eddy、mock 传感器包，见逐包表。 |
| ros2_ws/src/functions/perception/ | 视觉 ROS wrapper、算法安装适配包、D405 ROS wrapper。 |
| ros2_ws/src/functions/localization/ | FAST-LIVO2、vikit、Livox IMU 适配包。 |
| ros2_ws/src/functions/data/ | inspection_sync、inspection_tools。 |
| ros2_ws/src/functions/communication/ | agv_bridge、agv_protocol、RealSense MQTT。 |
| ros2_ws/src/functions/visualization/ | inspection_rerun、RealSense RViz 插件。 |
| ros2_ws/src/interfaces/ | 接口包；eddy_driver 内已有消息留在原包，禁止为目录整齐而拆包。 |
| ros2_ws/src/bringup/ | inspection_bringup、top_pkg。 |
| config/ | 站点/运行配置：system_websocket/config → config/system_websocket；physic_bev/config → config/data；tank_vision_inspection/configs → config/vision；Camera/Data 与 physic_bev/Camera/Data → config/camera 对应来源子目录（确认重复内容后再去重）。ROS 包自带 config/launch/resource 保持包内边界并安装到 share。 |
| deploy/ | tank_inspection_system 的部署清单、offline 清单/只读归档引用 → deploy/tank_inspection_system/；环境重建输出单独放 .runtime，不能写入共享依赖。 |
| tools/ | tank_inspection_system/scripts → tools/system；physic_bev/scripts → tools/data；视觉现场/校准/回放工具 → tools/vision。camera_collect.py → tools/camera/camera_collect.py。 |
| experiments/ | 视觉 experiment_*.py、training/、D405 实验配置 configs/、实验文档和 hello.py/零散探索材料经逐项分类后迁入；当前不批量改名。 |
| tests/ | 跨模块/系统验收：tank_inspection_system/tests、physic_bev/tests、websocket_console/tests 等按域迁入；ROS 包内部 test/ 和 Python 分发内 tests/ 保持在所属包中。 |
| docs/ | 根操作手册、工作记录与各模块说明按域归档；M0 三份文档位于 docs/architecture；历史 reports/refactor_v1 保留为历史证据，不重写其中旧哈希/成功状态。 |
| third_party/ | 保留 Livox-SDK2、Sophus、librealsense；两套 MindVision SDK 拟整理到 third_party/mindvision/<版本>/，头文件/绑定与对应原生库成套引用。RealSense 上游 LICENSE/NOTICE 等来源信息须随拆分映射保留。 |

## ROS 包完整清单

`package.xml` 是包名和声明依赖的依据；“目标”为规划。`sophus`、`librealsense2` 是第三方 CMake 项目，保留在 third_party，不作为业务 ROS 包搬入统一 src。忽略标记随所属包保留。

| 实际包名 | 当前源码目录 | 目标位置 | 构建类型/发现说明 |
|---|---|---|---|
| `d405_seam_ros` | `d405_seam_tracking/ros2_ws/src/d405_seam_ros` | `ros2_ws/src/functions/perception/d405_seam_ros` | ament_python |
| `fast_livo` | `physic_bev/src/FAST-LIVO2` | `ros2_ws/src/functions/localization/fast_livo` | ament_cmake |
| `eddy_driver` | `physic_bev/src/eddy_driver` | `ros2_ws/src/drivers/eddy_driver` | ament_cmake |
| `fisheye_camera_driver` | `physic_bev/src/fisheye_camera_driver` | `ros2_ws/src/drivers/fisheye_camera_driver` | ament_python |
| `inspection_bringup` | `physic_bev/src/inspection_bringup` | `ros2_ws/src/bringup/inspection_bringup` | ament_cmake |
| `inspection_interfaces` | `physic_bev/src/inspection_interfaces` | `ros2_ws/src/interfaces/inspection_interfaces` | ament_cmake |
| `inspection_livox_adapter` | `physic_bev/src/inspection_livox_adapter` | `ros2_ws/src/functions/localization/inspection_livox_adapter` | ament_cmake |
| `inspection_mock_sensors` | `physic_bev/src/inspection_mock_sensors` | `ros2_ws/src/drivers/inspection_mock_sensors` | ament_cmake |
| `inspection_sync` | `physic_bev/src/inspection_sync` | `ros2_ws/src/functions/data/inspection_sync` | ament_cmake |
| `inspection_tools` | `physic_bev/src/inspection_tools` | `ros2_ws/src/functions/data/inspection_tools` | ament_python |
| `livox_ros_driver2` | `physic_bev/src/livox_ros_driver2` | `ros2_ws/src/drivers/livox_ros_driver2` | ament_cmake |
| `mindvision_camera_driver` | `physic_bev/src/mindvision_camera_driver` | `ros2_ws/src/drivers/mindvision_camera_driver` | ament_python |
| `realsense2_camera` | `physic_bev/src/realsense-ros/realsense2_camera` | `ros2_ws/src/drivers/realsense-ros/realsense2_camera` | ament_cmake |
| `realsense2_camera_msgs` | `physic_bev/src/realsense-ros/realsense2_camera_msgs` | `ros2_ws/src/interfaces/realsense2_camera_msgs` | ament_cmake |
| `realsense2_description` | `physic_bev/src/realsense-ros/realsense2_description` | `ros2_ws/src/drivers/realsense-ros/realsense2_description` | ament_cmake |
| `realsense2_rviz_plugin` | `physic_bev/src/realsense-ros/realsense2_rgbd_plugin` | `ros2_ws/src/functions/visualization/realsense2_rgbd_plugin` | ament_cmake；CATKIN_IGNORE，未发现 |
| `realsense2_ros_mqtt_bridge` | `physic_bev/src/realsense-ros/realsense2_ros_mqtt_bridge` | `ros2_ws/src/functions/communication/realsense2_ros_mqtt_bridge` | ament_python；CATKIN_IGNORE，未发现 |
| `vikit_common` | `physic_bev/src/rpg_vikit/vikit_common` | `ros2_ws/src/functions/localization/rpg_vikit/vikit_common` | 未声明；colcon 实测 ros.catkin |
| `vikit_py` | `physic_bev/src/rpg_vikit/vikit_py` | `ros2_ws/src/functions/localization/rpg_vikit/vikit_py` | ament_cmake |
| `vikit_ros` | `physic_bev/src/rpg_vikit/vikit_ros` | `ros2_ws/src/functions/localization/rpg_vikit/vikit_ros` | ament_cmake |
| `agv_bridge` | `physic_bev/src/system_websocket/agv_bridge` | `ros2_ws/src/functions/communication/agv_bridge` | ament_cmake |
| `agv_protocol` | `physic_bev/src/system_websocket/agv_protocol` | `ros2_ws/src/functions/communication/agv_protocol` | ament_cmake |
| `top_pkg` | `physic_bev/src/top_pkg` | `ros2_ws/src/bringup/top_pkg` | ament_cmake |
| `inspection_rerun` | `tank_vision_inspection/ros2_ws/src/inspection_rerun` | `ros2_ws/src/functions/visualization/inspection_rerun` | ament_python |
| `tank_vision_core` | `tank_vision_inspection/ros2_ws/src/tank_vision_core` | `ros2_ws/src/functions/perception/tank_vision_core` | ament_cmake |
| `tank_vision_interfaces` | `tank_vision_inspection/ros2_ws/src/tank_vision_interfaces` | `ros2_ws/src/interfaces/tank_vision_interfaces` | ament_cmake |
| `tank_vision_ros` | `tank_vision_inspection/ros2_ws/src/tank_vision_ros` | `ros2_ws/src/functions/perception/tank_vision_ros` | ament_python |
| `sophus` | `third_party/Sophus` | `third_party/Sophus` | cmake；第三方项目，未纳入三源码根扫描 |
| `librealsense2` | `third_party/librealsense` | `third_party/librealsense` | cmake；第三方项目，未纳入三源码根扫描 |

## package.xml 依赖逐包摘录

下表保留依赖标签，未做 rosdep 解析或把未安装依赖认定为可用。`depend` 同时具有构建/导出/执行含义；重复声明在表中按原样保留。依赖声明不足项见审计 P14。

### d405_seam_ros

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_python` |
| `exec_depend` | `rclpy`, `sensor_msgs`, `std_msgs`, `nav_msgs`, `geometry_msgs`, `std_srvs`, `realsense2_camera_msgs`, `launch_ros` |

### fast_livo

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `depend` | `fmt` |
| `build_depend` | `rclcpp`, `rclpy`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `nav_msgs`, `std_msgs`, `tf2_ros`, `pcl_ros`, `pcl_conversions`, `livox_ros_driver2`, `vikit_common`, `vikit_ros`, `cv_bridge`, `image_transport`, `libopencv-dev` |
| `exec_depend` | `cv_bridge`, `image_transport`, `libopencv-dev`, `sensor_msgs`, `std_msgs` |
| `test_depend` | `ament_lint_auto`, `ament_lint_common` |

### eddy_driver

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `depend` | `rclcpp`, `std_msgs` |
| `build_depend` | `rosidl_default_generators` |
| `exec_depend` | `rosidl_default_runtime`, `launch`, `launch_ros`, `ament_index_python` |

### fisheye_camera_driver

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_python` |
| `exec_depend` | `diagnostic_msgs`, `python3-opencv`, `python3-yaml`, `rclpy`, `sensor_msgs` |
| `test_depend` | `python3-pytest` |

### inspection_bringup

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `exec_depend` | `ament_index_python`, `eddy_driver`, `fisheye_camera_driver`, `inspection_mock_sensors`, `inspection_livox_adapter`, `inspection_sync`, `inspection_tools`, `launch`, `launch_ros`, `livox_ros_driver2`, `mindvision_camera_driver`, `realsense2_camera`, `python3-yaml`, `ros2bag`, `tf2_ros` |

### inspection_interfaces

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `build_depend` | `rosidl_default_generators` |
| `depend` | `builtin_interfaces`, `geometry_msgs`, `std_msgs` |
| `exec_depend` | `rosidl_default_runtime` |

### inspection_livox_adapter

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `depend` | `diagnostic_msgs`, `rclcpp`, `sensor_msgs` |
| `test_depend` | `ament_cmake_gtest` |

### inspection_mock_sensors

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `depend` | `geometry_msgs`, `inspection_interfaces`, `nav_msgs`, `rclcpp`, `sensor_msgs`, `tf2_ros` |

### inspection_sync

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `depend` | `diagnostic_msgs`, `geometry_msgs`, `inspection_interfaces`, `eddy_driver`, `nav_msgs`, `rclcpp`, `sensor_msgs` |
| `test_depend` | `ament_cmake_gtest` |

### inspection_tools

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_python` |
| `exec_depend` | `inspection_interfaces`, `python3-numpy`, `python3-yaml`, `rclpy`, `rosbag2_py`, `rosidl_runtime_py`, `sensor_msgs`, `diagnostic_msgs`, `eddy_driver` |
| `test_depend` | `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest` |

### livox_ros_driver2

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake_auto` |
| `build_depend` | `rosidl_default_generators` |
| `depend` | `rclcpp`, `rclcpp_components`, `std_msgs`, `sensor_msgs`, `diagnostic_msgs`, `rcutils`, `pcl_conversions`, `rcl_interfaces`, `libpcl-all-dev`, `git`, `apr` |
| `exec_depend` | `rosbag2`, `rosidl_default_runtime` |
| `test_depend` | `ament_lint_auto`, `ament_lint_common` |

### mindvision_camera_driver

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_python` |
| `exec_depend` | `ament_index_python`, `builtin_interfaces`, `python3-numpy`, `python3-opencv`, `diagnostic_msgs`, `rclpy`, `sensor_msgs` |
| `test_depend` | `python3-pytest` |

### realsense2_camera

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `depend` | `eigen`, `builtin_interfaces`, `cv_bridge`, `image_transport`, `librealsense2`, `rclcpp`, `rclcpp_lifecycle`, `lifecycle_msgs`, `rclcpp_components`, `realsense2_camera_msgs`, `sensor_msgs`, `geometry_msgs`, `std_srvs`, `std_msgs`, `nav_msgs`, `tf2`, `tf2_ros`, `diagnostic_updater` |
| `test_depend` | `ament_cmake_gtest`, `launch_testing`, `ament_cmake_pytest`, `launch_pytest`, `sensor_msgs_py`, `python3-numpy`, `python3-tqdm`, `sensor_msgs_py`, `python3-requests`, `tf2_ros_py`, `ros2topic` |
| `exec_depend` | `launch_ros` |
| `build_depend` | `ros_environment` |

### realsense2_camera_msgs

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake`, `rosidl_default_generators` |
| `build_depend` | `action_msgs`, `builtin_interfaces`, `std_msgs`, `sensor_msgs` |
| `exec_depend` | `rosidl_default_runtime`, `action_msgs`, `builtin_interfaces`, `std_msgs`, `sensor_msgs` |
| `test_depend` | `ament_lint_common` |

### realsense2_description

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `depend` | `rclcpp`, `rclcpp_components`, `realsense2_camera_msgs` |
| `exec_depend` | `launch_ros`, `xacro` |

### realsense2_rviz_plugin

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `depend` | `rclcpp`, `sensor_msgs`, `realsense2_camera_msgs`, `rviz_common`, `rviz_rendering`, `rviz_default_plugins`, `pluginlib` |

### realsense2_ros_mqtt_bridge

| 原标签 | 原文包名 |
|---|---|
| `test_depend` | `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest` |

### vikit_common

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `build_depend` | `rclcpp`, `cmake_modules` |
| `exec_depend` | `rclcpp` |

### vikit_py

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `depend` | `rclcpp`, `rclpy` |
| `test_depend` | `ament_lint_auto`, `ament_lint_common` |

### vikit_ros

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake`, `rosidl_default_generators` |
| `depend` | `rclcpp`, `cmake_modules`, `vikit_common`, `tf2`, `visualization_msgs` |
| `exec_depend` | `rosidl_default_runtime` |

### agv_bridge

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `depend` | `rclcpp`, `geometry_msgs`, `std_msgs`, `nav_msgs`, `sensor_msgs`, `agv_protocol`, `eddy_driver`, `inspection_interfaces`, `libopencv-dev`, `openssl` |
| `test_depend` | `ament_cmake_pytest`, `python3-websockets` |

### agv_protocol

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `depend` | `rclcpp`, `jsoncpp`, `boost` |
| `test_depend` | `ament_lint_auto`, `ament_lint_common` |

### top_pkg

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `exec_depend` | `launch`, `launch_ros`, `ament_index_python`, `python3-yaml`, `fast_livo`, `livox_ros_driver2`, `realsense2_camera`, `rviz2` |

### inspection_rerun

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_python` |
| `depend` | `ament_index_python`, `diagnostic_msgs`, `geometry_msgs`, `nav_msgs`, `rclpy`, `sensor_msgs`, `tf2_msgs` |
| `exec_depend` | `inspection_tools`, `python3-numpy`, `python3-yaml` |
| `test_depend` | `python3-pytest` |

### tank_vision_core

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake`, `ament_cmake_python` |
| `exec_depend` | `python3-numpy`, `python3-opencv`, `python3-yaml` |

### tank_vision_interfaces

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_cmake` |
| `build_depend` | `rosidl_default_generators` |
| `depend` | `geometry_msgs`, `sensor_msgs`, `std_msgs` |
| `exec_depend` | `rosidl_default_runtime` |

### tank_vision_ros

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `ament_python` |
| `depend` | `ament_index_python`, `cv_bridge`, `diagnostic_msgs`, `geometry_msgs`, `rclpy`, `sensor_msgs`, `std_srvs`, `tank_vision_core`, `tank_vision_interfaces` |
| `exec_depend` | `python3-opencv`, `python3-yaml` |

### sophus

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `cmake` |
| `build_depend` | `eigen` |
| `build_export_depend` | `eigen` |

### librealsense2

| 原标签 | 原文包名 |
|---|---|
| `buildtool_depend` | `cmake` |
| `build_depend` | `pkg-config`, `libusb-1.0-dev`, `libssl-dev`, `libudev-dev`, `dkms`, `git`, `udev` |
| `depend` | `libx11`, `libglfw3-dev`, `opengl` |

## M1 第一批迁移方案

### 范围与次序

第一批仅迁移 5 个完整基础包，先接口和 Eddy，再 IMU 适配、同步器及 mock。共 36 个文件，不拆包、不更改消息定义，也不把 eddy_driver 自带消息独立出去。算法、厂商 SDK、应用和工具目录的物理迁移留在后续批次；本批需修改仍留在旧目录中的引用消费者。

| 计划移动的现有文件 | 目标文件 |
|---|---|
| `physic_bev/src/inspection_interfaces/CMakeLists.txt` | `ros2_ws/src/interfaces/inspection_interfaces/CMakeLists.txt` |
| `physic_bev/src/inspection_interfaces/msg/EddyCurrentFrame.msg` | `ros2_ws/src/interfaces/inspection_interfaces/msg/EddyCurrentFrame.msg` |
| `physic_bev/src/inspection_interfaces/msg/FusionIndex.msg` | `ros2_ws/src/interfaces/inspection_interfaces/msg/FusionIndex.msg` |
| `physic_bev/src/inspection_interfaces/msg/ProbeState.msg` | `ros2_ws/src/interfaces/inspection_interfaces/msg/ProbeState.msg` |
| `physic_bev/src/inspection_interfaces/msg/RobotState.msg` | `ros2_ws/src/interfaces/inspection_interfaces/msg/RobotState.msg` |
| `physic_bev/src/inspection_interfaces/msg/UltrasoundFrame.msg` | `ros2_ws/src/interfaces/inspection_interfaces/msg/UltrasoundFrame.msg` |
| `physic_bev/src/inspection_interfaces/package.xml` | `ros2_ws/src/interfaces/inspection_interfaces/package.xml` |
| `physic_bev/src/eddy_driver/CMakeLists.txt` | `ros2_ws/src/drivers/eddy_driver/CMakeLists.txt` |
| `physic_bev/src/eddy_driver/README.md` | `ros2_ws/src/drivers/eddy_driver/README.md` |
| `physic_bev/src/eddy_driver/config/eddy.yaml` | `ros2_ws/src/drivers/eddy_driver/config/eddy.yaml` |
| `physic_bev/src/eddy_driver/include/eddy_driver/eddy_driver.hpp` | `ros2_ws/src/drivers/eddy_driver/include/eddy_driver/eddy_driver.hpp` |
| `physic_bev/src/eddy_driver/include/eddy_driver/eddy_node.hpp` | `ros2_ws/src/drivers/eddy_driver/include/eddy_driver/eddy_node.hpp` |
| `physic_bev/src/eddy_driver/launch/eddy_driver.launch.py` | `ros2_ws/src/drivers/eddy_driver/launch/eddy_driver.launch.py` |
| `physic_bev/src/eddy_driver/msg/EddyChannelMean.msg` | `ros2_ws/src/drivers/eddy_driver/msg/EddyChannelMean.msg` |
| `physic_bev/src/eddy_driver/msg/EddyCommand.msg` | `ros2_ws/src/drivers/eddy_driver/msg/EddyCommand.msg` |
| `physic_bev/src/eddy_driver/msg/EddyFrame.msg` | `ros2_ws/src/drivers/eddy_driver/msg/EddyFrame.msg` |
| `physic_bev/src/eddy_driver/msg/EddyStatus.msg` | `ros2_ws/src/drivers/eddy_driver/msg/EddyStatus.msg` |
| `physic_bev/src/eddy_driver/package.xml` | `ros2_ws/src/drivers/eddy_driver/package.xml` |
| `physic_bev/src/eddy_driver/src/eddy_driver.cpp` | `ros2_ws/src/drivers/eddy_driver/src/eddy_driver.cpp` |
| `physic_bev/src/eddy_driver/src/eddy_main.cpp` | `ros2_ws/src/drivers/eddy_driver/src/eddy_main.cpp` |
| `physic_bev/src/eddy_driver/src/eddy_node.cpp` | `ros2_ws/src/drivers/eddy_driver/src/eddy_node.cpp` |
| `physic_bev/src/eddy_driver/udev/99-eddy.rules` | `ros2_ws/src/drivers/eddy_driver/udev/99-eddy.rules` |
| `physic_bev/src/inspection_livox_adapter/CMakeLists.txt` | `ros2_ws/src/functions/localization/inspection_livox_adapter/CMakeLists.txt` |
| `physic_bev/src/inspection_livox_adapter/include/inspection_livox_adapter/imu_conversion.hpp` | `ros2_ws/src/functions/localization/inspection_livox_adapter/include/inspection_livox_adapter/imu_conversion.hpp` |
| `physic_bev/src/inspection_livox_adapter/package.xml` | `ros2_ws/src/functions/localization/inspection_livox_adapter/package.xml` |
| `physic_bev/src/inspection_livox_adapter/src/livox_imu_adapter_node.cpp` | `ros2_ws/src/functions/localization/inspection_livox_adapter/src/livox_imu_adapter_node.cpp` |
| `physic_bev/src/inspection_livox_adapter/test/test_imu_conversion.cpp` | `ros2_ws/src/functions/localization/inspection_livox_adapter/test/test_imu_conversion.cpp` |
| `physic_bev/src/inspection_sync/CMakeLists.txt` | `ros2_ws/src/functions/data/inspection_sync/CMakeLists.txt` |
| `physic_bev/src/inspection_sync/include/inspection_sync/sync_math.hpp` | `ros2_ws/src/functions/data/inspection_sync/include/inspection_sync/sync_math.hpp` |
| `physic_bev/src/inspection_sync/include/inspection_sync/timestamped_ring_buffer.hpp` | `ros2_ws/src/functions/data/inspection_sync/include/inspection_sync/timestamped_ring_buffer.hpp` |
| `physic_bev/src/inspection_sync/package.xml` | `ros2_ws/src/functions/data/inspection_sync/package.xml` |
| `physic_bev/src/inspection_sync/src/inspection_sync_node.cpp` | `ros2_ws/src/functions/data/inspection_sync/src/inspection_sync_node.cpp` |
| `physic_bev/src/inspection_sync/test/test_sync_core.cpp` | `ros2_ws/src/functions/data/inspection_sync/test/test_sync_core.cpp` |
| `physic_bev/src/inspection_mock_sensors/CMakeLists.txt` | `ros2_ws/src/drivers/inspection_mock_sensors/CMakeLists.txt` |
| `physic_bev/src/inspection_mock_sensors/package.xml` | `ros2_ws/src/drivers/inspection_mock_sensors/package.xml` |
| `physic_bev/src/inspection_mock_sensors/src/mock_sensors_node.cpp` | `ros2_ws/src/drivers/inspection_mock_sensors/src/mock_sensors_node.cpp` |

### 必须与本批同步修改的引用

这些是本批源码移动之外的编辑清单；行号对应 M0 提交。旧 reports 中的路径属于历史证据，只在新映射中解释，不做全仓字符串替换。

| 现有文件与定位 | 同步修改内容 |
|---|---|
| `tank_inspection_system/scripts/build.sh:44` | 4 个 cpp `--base-paths` 改到新目标；本批单独构建入口纳入 inspection_mock_sensors。保留其余未迁包源码根，不误指新目录。检查 source 顺序使旧位置未迁消费者使用新 cpp overlay。 |
| `physic_bev/scripts/start_vehicle_collection.sh:250-251` | freeze-file 的 eddy_driver/msg/EddyChannelMean.msg 与 src/eddy_node.cpp 改到新位置；保持采集 run 内冻结文件格式和只读校验规则。 |
| `physic_bev/config/data_handoff_profiles.yaml:89-92` | eddy profile 的 README、package.xml、config、msg 源码路径改到新包。该文件虽扩展名 yaml，内容为 JSON，现有加载方式必须保持。 |
| `physic_bev/scripts/build_role_handoff.py:376-377,439,759-771,992` | 更新接口来源文字、Eddy 构建示例、package_paths、自定义接口路径和角色包生成用映射；旧相对路径不能再以 PROJECT_ROOT/physic_bev 拼接。未迁 fast_livo/Livox/RealSense 路径继续有效。 |
| `tank_inspection_system/manifests/entrypoints.json` | 本批包 packages/entries 的 source、python_imports 的 consumer、config_authorities 的 path 与 C++ 参数 source 字段同步核对更新。由当前源码重建活跃清单或定向修改路径；不将历史快照里的包/依赖列表当作完整盘点。 |
| `tank_inspection_system/manifests/topic_contract.yaml:107,172,182,194,206,218,231` | 源码溯源路径同步；保持原消息类型与 topic。 |
| `physic_bev/src/inspection_tools/config/stability_config.yaml:179` | 时间戳证据 code 路径更新到 ros2_ws/src/drivers/eddy_driver/src/eddy_driver.cpp；行号按迁移后实际核对。 |
| `tank_inspection_system/tests/ros_native_eddy_e2e.py:29` | 旧 install_cpp/... 可执行文件绝对拼接改为 get_package_prefix(inspection_sync)/lib/...，使合成验证实际运行本批新安装的节点。不改预期消息计数/质量门限。 |
| 本轮 `docs/architecture/*.md`、`README.md`、`tank_inspection_system/docs/module_development_and_debugging.md` 等现行入口说明 | 记录本批实际完成的移动和构建方法；与当前构建入口有关的旧路径示例逐项更新，保留日期明确的历史记录。 |

`inspection_sync`、`inspection_mock_sensors`、`eddy_driver`、`inspection_livox_adapter` 的 CMake 安装均以包内相对路径和 `lib/${PROJECT_NAME}` 为主，本批不需要修改其包名、target 名或 ament share 名。`inspection_bringup` 的 launch、`agv_bridge` 等按包名引用这些包，可继续留在旧源码目录，但必须重建/source 正确 overlay。接口 Python 导入路径保持 `inspection_interfaces.msg`、`eddy_driver.msg`。

`tank_inspection_system/manifests/repositories.json` 保存的是旧仓库命令输出快照，其中旧路径不属于当前配置 authority；应保留原证据，另记录本次顶层仓库身份。`reports/refactor_v1/` 下的路径、哈希与测试状态同样不得随源码迁移全局改写。

### 构建与功能验证（M1 执行，M0 未执行）

先核对 ament_cmake、rosidl_default_generators、ament_cmake_gtest、rclcpp、tf2_ros 及相关消息包在干净 Humble 环境可发现。第一批不需要载入旧工程 overlay、TROS 或厂商 SDK。以下命令以第一批已按表迁移为前提；任一步失败必须保留退出码与日志，不继续宣称验收通过。

```bash
cd /media/mcap_data/project_east_eleair_refactor
# 在新终端清除继承的项目 overlay 和 venv，使用系统工具链。
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
source /opt/ros/humble/setup.bash
colcon --log-base ros2_ws/log_m1_batch1 build \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build_m1_batch1 \
  --install-base ros2_ws/install_m1_batch1 \
  --packages-select inspection_interfaces eddy_driver inspection_livox_adapter inspection_sync inspection_mock_sensors \
  --cmake-args -DBUILD_TESTING=ON
source ros2_ws/install_m1_batch1/setup.bash
colcon --log-base ros2_ws/log_m1_batch1_test test \
  --base-paths ros2_ws/src --build-base ros2_ws/build_m1_batch1 \
  --install-base ros2_ws/install_m1_batch1 \
  --packages-select inspection_interfaces eddy_driver inspection_livox_adapter inspection_sync inspection_mock_sensors \
  --return-code-on-test-failure
colcon test-result --test-result-base ros2_ws/build_m1_batch1 --verbose
```

| 验证对象 | 方法 | 通过依据 |
|---|---|---|
| 包发现和前缀 | colcon list --base-paths ros2_ws/src；ros2 pkg prefix 对 5 包逐一检查；ros2 pkg executables | 包名无重复；前缀均在新 install_m1_batch1，不落入旧工程；节点可执行文件存在。 |
| 消息 ABI/类型 | python3 导入 inspection_interfaces.msg 的 FusionIndex、EddyCurrentFrame、RobotState、ProbeState、UltrasoundFrame，以及 eddy_driver.msg 的 EddyFrame、EddyChannelMean、EddyCommand、EddyStatus；ros2 interface show | 类型名与字段逐项对比原 msg；只验证类型生成，不表示串口已验收。 |
| 基础数学/缓冲逻辑 | colcon test 运行 inspection_sync/test/test_sync_core.cpp、inspection_livox_adapter/test/test_imu_conversion.cpp | 确有测试执行，全部断言通过；不能以 0 tests 当成通过。 |
| 同步真实 ROS 类型与 QoS | 修好测试二进制定位后运行 python3 tank_inspection_system/tests/ros_native_eddy_e2e.py --output /tmp/m1_batch1_eddy_e2e.json（目标文件事先不存在） | 脚本自带隔离域和合成输入；两类 Eddy anchor 各 5 条 FusionIndex、时间戳/序号/有效标记满足原断言，无硬件访问。记录实际执行结果。 |
| mock 发布链 | 在独立 ROS_DOMAIN_ID 和 ROS_LOCALHOST_ONLY=1 下运行 ros2 run inspection_mock_sensors mock_sensors_node；使用传感器 QoS 订阅相机/LiDAR/IMU/里程计及 inspection 原始消息，有限时长后停止 | 消息可接收，frame_id/时间戳/类型正确；FusionIndex 端到端仍以上一项断言为准。 |
| Eddy 启动资源 | ros2 launch eddy_driver eddy_driver.launch.py --show-args；确认新 share 中 config/eddy.yaml、launch 文件存在 | 资源定位指向新 install；--show-args 不是实际启动/串口验收。 |
| 角色交付与冻结源码 | 运行经更新的 wrapper/role-handoff 测试，至少覆盖 eddy profile 的源码和 config/msg 清单；使用临时 run fixture | 不再读取已迁走的 src/eddy_driver；封存 run、历史校验和不被改写。测试依赖若需 inspection_tools，先从仍存在的旧源码位置构建到新的独立输出。 |
| 无旧路径依赖 | 检查新 install 的 Python __file__、CMakeCache、导出 cmake、启动资源；对比迁移前后 36 文件内容 SHA（纯移动文件应相同） | 不存在指向 /media/mcap_data/east_eleair_plus 的项目路径；无旧 build/install/venv 参与；源定义无意外改动。 |
| 硬件回归（后续） | 依赖恢复且设备具备后，Eddy 串口协议、实际相机/LiDAR、采集 MCAP、回放、WebSocket 全链路另做记录 | 独立报告真实数据、命令、前缀、退出码；M0 和无硬件 M1 测试不能替代。 |

全量迁移到统一 ros2_ws 后另需验证 fast_livo/vikit 的链接与运行资源，两个忽略包保持原启用策略；本批构建通过不代表全量 workspace 或整机通过。
