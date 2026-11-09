# 已实施迁移映射

本表记录全量迁移后的布局，取代 M0/M1 分批计划；M0 原表保存于 `docs/history/m0/migration_map.md`。
基线 `b34a3c29c62500915c302b340cb4d51880b00980` 的 5,685 个 Git 文件逐项映射见 `migration_files.json`；
另从原工程只读恢复 5 个原本被忽略的 `tank_vision.outputs` 源文件，来源和 SHA 见 `migration_supplemental_files.json`。
所有路径相对仓库根。没有以软链接旧工作树维持运行。

## ROS 包

| 实际包名（保持） | 原源码目录 | 迁移后目录 |
|---|---|---|
| `d405_seam_ros` | `d405_seam_tracking/ros2_ws/src/d405_seam_ros` | `ros2_ws/src/functions/perception/d405_seam_ros` |
| `fast_livo` | `physic_bev/src/FAST-LIVO2` | `ros2_ws/src/functions/localization/FAST-LIVO2` |
| `eddy_driver` | `physic_bev/src/eddy_driver` | `ros2_ws/src/drivers/eddy_driver` |
| `fisheye_camera_driver` | `physic_bev/src/fisheye_camera_driver` | `ros2_ws/src/drivers/fisheye_camera_driver` |
| `inspection_bringup` | `physic_bev/src/inspection_bringup` | `ros2_ws/src/bringup/inspection_bringup` |
| `inspection_interfaces` | `physic_bev/src/inspection_interfaces` | `ros2_ws/src/interfaces/inspection_interfaces` |
| `inspection_livox_adapter` | `physic_bev/src/inspection_livox_adapter` | `ros2_ws/src/drivers/inspection_livox_adapter` |
| `inspection_mock_sensors` | `physic_bev/src/inspection_mock_sensors` | `ros2_ws/src/drivers/inspection_mock_sensors` |
| `inspection_sync` | `physic_bev/src/inspection_sync` | `ros2_ws/src/functions/data/inspection_sync` |
| `inspection_tools` | `physic_bev/src/inspection_tools` | `ros2_ws/src/functions/data/inspection_tools` |
| `livox_ros_driver2` | `physic_bev/src/livox_ros_driver2` | `ros2_ws/src/drivers/livox_ros_driver2` |
| `mindvision_camera_driver` | `physic_bev/src/mindvision_camera_driver` | `ros2_ws/src/drivers/mindvision_camera_driver` |
| `realsense2_camera` | `physic_bev/src/realsense-ros/realsense2_camera` | `ros2_ws/src/drivers/realsense-ros/realsense2_camera` |
| `realsense2_camera_msgs` | `physic_bev/src/realsense-ros/realsense2_camera_msgs` | `ros2_ws/src/drivers/realsense-ros/realsense2_camera_msgs` |
| `realsense2_description` | `physic_bev/src/realsense-ros/realsense2_description` | `ros2_ws/src/drivers/realsense-ros/realsense2_description` |
| `realsense2_rviz_plugin` | `physic_bev/src/realsense-ros/realsense2_rgbd_plugin` | `ros2_ws/src/drivers/realsense-ros/realsense2_rgbd_plugin` |
| `realsense2_ros_mqtt_bridge` | `physic_bev/src/realsense-ros/realsense2_ros_mqtt_bridge` | `ros2_ws/src/drivers/realsense-ros/realsense2_ros_mqtt_bridge` |
| `vikit_common` | `physic_bev/src/rpg_vikit/vikit_common` | `ros2_ws/src/functions/localization/rpg_vikit/vikit_common` |
| `vikit_py` | `physic_bev/src/rpg_vikit/vikit_py` | `ros2_ws/src/functions/localization/rpg_vikit/vikit_py` |
| `vikit_ros` | `physic_bev/src/rpg_vikit/vikit_ros` | `ros2_ws/src/functions/localization/rpg_vikit/vikit_ros` |
| `agv_bridge` | `physic_bev/src/system_websocket/agv_bridge` | `ros2_ws/src/functions/communication/agv_bridge` |
| `agv_protocol` | `physic_bev/src/system_websocket/agv_protocol` | `ros2_ws/src/functions/communication/agv_protocol` |
| `top_pkg` | `physic_bev/src/top_pkg` | `ros2_ws/src/bringup/top_pkg` |
| `inspection_rerun` | `tank_vision_inspection/ros2_ws/src/inspection_rerun` | `ros2_ws/src/functions/visualization/inspection_rerun` |
| `tank_vision_core` | `tank_vision_inspection/ros2_ws/src/tank_vision_core` | `ros2_ws/src/functions/perception/tank_vision_core` |
| `tank_vision_interfaces` | `tank_vision_inspection/ros2_ws/src/tank_vision_interfaces` | `ros2_ws/src/interfaces/tank_vision_interfaces` |
| `tank_vision_ros` | `tank_vision_inspection/ros2_ws/src/tank_vision_ros` | `ros2_ws/src/functions/perception/tank_vision_ros` |

新增 `inspection_operations` 位于 `ros2_ws/src/functions/communication/inspection_operations`，承接原 operations_node、replay_worker、监控、录像及 bag 读取适配。实际 colcon 发现 28 包，见 `docs/validation/colcon_discovery.txt`。
`FAST-LIVO2` 仍为 fast_livo；RealSense 整体上游目录与 5 包保留，rpg_vikit 整体目录与 3 包保留。
RealSense 两个可选包的 CATKIN_IGNORE 改名 `.upstream` 保存，以便全部可发现；是否构建由 `config/platforms/*.json` 明示。
第三方非 ROS 项目 sophus/librealsense2 的 package.xml 留在 third_party，统一构建仅扫描 ros2_ws/src，不误计入 28 包。
包依赖及路径的机器可读检查结果见 `docs/validation/migration_static_results.json`。原 msg/srv/action 内容哈希不变。

## 非 ROS 内容

| 原内容 | 唯一权威位置/职责 |
|---|---|
| websocket_console | apps/web_console；HTML/JS 资源与薄服务入口；测试在 tests/web_console |
| system_websocket 的 Operations、ReplayController、run_system | packages/inspection_runtime/src/inspection_runtime；supervisor 保留原进程编排，ROS 节点在 inspection_operations |
| physic_bev/scripts 的采集管理、校验、交付、历史 | inspection_runtime/data 与 scripts；tools/data 的同名兼容命令委托这些实现 |
| physic_bev/scripts 的录像、监控与 ROS 回放 | inspection_operations；业务层通过模块边界调用，必要 ROS 依赖不可用时明确报错 |
| tank_vision_inspection/tank_vision | packages/tank_vision/tank_vision；pip 所有权；tank_vision_core 仅依赖标记与来源检查，不再跨目录复制算法 |
| d405_seam_tracking/src | packages/d405_seam_tracking/src；ROS node 实现抽至 d405_seam_ros；原公共模块保留薄导入/调用兼容 |
| emat 协议及测厚 | packages/emat_core；runtime 调度，inspection_operations 发布波形及事件；其余历史实现与样例在 experiments/emat |
| SDK | third_party/mindvision/legacy 与 2.1.0.50；头文件/绑定/许可证随原目录保留，二进制忽略 Git |
| 原 offline | third_party/offline/system、d405、mindvision；清单受版本管理，归档忽略 Git |
| camera_collect.py、hello.py | tools/camera、experiments/examples |
| Camera、physic_bev/Camera | config/camera/root、config/camera/physic；保留不同来源，不推断标定有效性 |
| 视觉工具、实验/训练/数据集/小样例 | tools/vision、experiments/vision；模块测试仍随 packages/tank_vision |
| tank_inspection_system scripts/tests/reports | deploy、tools/system、tests/system、docs/history/reports；过时审计工具原文保存 docs/history/legacy_tools |
| 原工作记录/说明书/算法文档 | docs/history/root、docs/system、docs/data、docs/vision、docs/d405、docs/emat |

新 CLI 在 apps/inspection_cli，运行时分发通过 exec 保持参数/退出码/信号；run、collect、stop、status、replay、validate、handoff、verify-handoff、history 调用现有实现。
兼容包装集中在 deploy/compat，tools/data 保留必要开发命令转发；不重建旧顶层模块目录。完整名单见 `compatibility_manifest.json`。

## 配置与安装

中央入口为 config/README.md；独立业务配置迁入 config，ROS 包自带 config/launch/resource 仍由所属包安装到 share，只有一处源码权威。
视觉源配置由 tank_vision_ros/setup.py 从中央 config/vision 安装；安装副本是生成物，应重新构建更新，不手改。
Python 包可编辑安装保证单一算法来源。build 明确传入新 Python、native SDK 路径与 FAST-LIVO 输出路径。
已验证安装元数据声明的资源存在；CMake configure、完整安装/运行行为仍待下一轮构建。
