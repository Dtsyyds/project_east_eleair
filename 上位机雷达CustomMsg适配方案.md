# 实现内容：上位机原始雷达数据适配 Livox CustomMsg

将现有上位机的原始雷达链路从 `sensor_msgs/msg/PointCloud2` 改为 `livox_ros_driver2/msg/CustomMsg`，实现雷达状态检测、采集预检、完整 MCAP 录制及新旧格式录包回放。

**状态：已实施并构建到当前系统；软件链路验证见 [验收记录](docs/validation/20261109_livox_custommsg.md)。真实雷达联调待 eth0 链路恢复后完成。**

## 1. 当前问题与实施范围

- 驱动独立启动配置已经设置 `xfer_format: 1`，但上位机硬件启动入口仍强制设置为 `0`，实际启动时会继续输出 PointCloud2。
- 雷达状态检测和采集预检仍订阅 PointCloud2，不能直接接收 CustomMsg。
- 雷达回放仍按 PointCloud2 的 `fields`、`point_step` 和 `data` 解析，新录包需要增加 CustomMsg 分支。
- 实施时确认当前页面已有实时点云窗口及 R6 磁吸附车模。本次将现有窗口的后端数据源适配为 CustomMsg，沿用页面布局、车模及显示坐标关系，不创建新的上位机目录。

以 `system_websocket/run_robot.sh` 整合入口、默认 `start_sync=false` 的链路为验收范围。独立 FIELD 采集脚本、inspection_sync 和 Rerun 的格式迁移不在本次范围内。

## 2. 数据与接口约定

| 项目 | 约定 |
|---|---|
| 原始雷达话题 | `/livox/lidar`，名称保持不变 |
| 实时订阅类型 | `livox_ros_driver2/msg/CustomMsg` |
| 帧时间 | 使用有效的 `header.stamp`；保留整数纳秒 `timebase` |
| 原始点 | `points[]` 中的 XYZ 单位为米；保留 `offset_time`、`reflectivity`、`tag`、`line` |
| 消息数量 | `point_num > 0`，且等于 `points` 数组长度 |
| 录制 | 原样录制完整 CustomMsg；不转换、不降采样 |
| 回放预览 | 支持 CustomMsg 和历史 PointCloud2，沿用现有 XYZ 浏览器协议 |
| 算法输出 | `/cloud_registered`、`/cloud_effected` 等继续使用 PointCloud2 |

不得全工程替换 PointCloud2，也不增加 CustomMsg 转 PointCloud2 的中间节点。IMU、设备网络 JSON、对外端口和现有页面布局沿用现状。

## 3. 修改文件与实现要求

以下链接指向当前工程；在其他机器实施时，使用对应工程根目录下的同名文件。

### 3.1 修正上位机启动参数

修改 [hardware_pipeline.launch.py](/home/fire/Desktop/project_east_eleair/ros2_ws/src/bringup/inspection_bringup/launch/hardware_pipeline.launch.py)：

- 将 `_start_mid360()` 内驱动参数 `xfer_format` 从 `0` 改为 `1`。
- 将启动日志、参数说明中的 PointCloud2 描述同步改为 Livox CustomMsg。
- 保留现有话题重映射、发布频率、坐标系及 IMU 适配逻辑。
- 不改为包含另一套驱动启动文件，避免重复启动雷达或改变现有设备配置。

### 3.2 适配雷达设备状态

修改 [tank_inspection_robot.cpp](/home/fire/Desktop/project_east_eleair/ros2_ws/src/functions/communication/agv_bridge/src/tank_inspection_robot.cpp)：

- 引入 `livox_ros_driver2/msg/custom_msg.hpp`，将原始雷达订阅改为 `livox_ros_driver2::msg::CustomMsg`。
- 将雷达状态中的 `message_type` 改为 `livox_ros_driver2/msg/CustomMsg`。
- 用点数一致、非空、`timebase > 0` 的检查替代旧 PointCloud2 布局检查。
- 继续通过现有 `mark()` 判断 header 时间戳有效性、重复、倒退与数据新鲜度；结构无效时使用明确原因，例如 `invalid_livox_custom_msg`。
- 沿用现有传感器 QoS、状态字段和计数方式。

修改 [agv_bridge/CMakeLists.txt](/home/fire/Desktop/project_east_eleair/ros2_ws/src/functions/communication/agv_bridge/CMakeLists.txt) 和 [agv_bridge/package.xml](/home/fire/Desktop/project_east_eleair/ros2_ws/src/functions/communication/agv_bridge/package.xml)，补齐 `livox_ros_driver2` 的查找、目标依赖及包依赖。

### 3.3 适配采集预检

修改 [operations_node.py](/home/fire/Desktop/project_east_eleair/ros2_ws/src/functions/communication/inspection_operations/inspection_operations/operations_node.py)：

- 将必需话题 `/livox/lidar` 的订阅类型改为 CustomMsg。
- 有效帧才能更新该话题的有效数据接收时间。
- 无效帧不刷新时间；持续无效或断流后，现有超时机制应使预检失败。
- 不放宽已有存储、其他设备与采集启动条件。

修改 [inspection_operations/package.xml](/home/fire/Desktop/project_east_eleair/ros2_ws/src/functions/communication/inspection_operations/package.xml)，增加 `livox_ros_driver2` 运行依赖。

### 3.4 增加统一校验与录包类型分支

修改 [validators.py](/home/fire/Desktop/project_east_eleair/ros2_ws/src/functions/data/inspection_tools/inspection_tools/validators.py)，增加 `validate_livox_custom(message)`：

- 检查 `point_num > 0`、`point_num == len(points)` 和 `timebase > 0`。
- 检查 `header.stamp` 为正、秒数非负且纳秒部分在 `[0, 1000000000)`。
- 返回错误列表，与现有校验器风格一致，供 Python 预检、回放与录包校验复用。
- 不因个别点的 XYZ 非有限就删除或改写原始录制内容；这类点只在预览时过滤。

修改 [validate_bag.py](/home/fire/Desktop/project_east_eleair/ros2_ws/src/functions/data/inspection_tools/inspection_tools/validate_bag.py)：

- 根据录包记录的实际消息类型选择 CustomMsg 或原 PointCloud2 校验器。
- 未支持的雷达类型返回明确错误，不能只根据话题名称套用解析器。

修改 [inspection_tools/package.xml](/home/fire/Desktop/project_east_eleair/ros2_ws/src/functions/data/inspection_tools/package.xml)，补充 `livox_ros_driver2` 运行依赖。

### 3.5 保持录制完整，兼容新旧回放

现有网页采集使用 `ros2 bag record` 按话题录制，可直接记录 CustomMsg。保留当前录制流程，验收时核对实际消息类型及原始字段，无需新增转换链路。

修改 [replay_worker.py](/home/fire/Desktop/project_east_eleair/ros2_ws/src/functions/communication/inspection_operations/inspection_operations/replay_worker.py)：

- 按实际反序列化后的消息类型选择解析分支，保留原 PointCloud2 解析函数。
- 新增 CustomMsg 预览函数，先校验结构，再从 `points[]` 读取 XYZ。
- 沿用现有浏览器协议：`kind: "cloud"`、`xyz`、`point_count`、`displayed_points`、`unit: "m"`。
- `point_count` 表示原始点数；`displayed_points` 表示实际预览点数。
- 均匀采样后每帧最多显示 5000 点，过滤 NaN/Inf；处理仅作用于预览，不修改消息和录包。
- 保留当前回放节奏、暂停、停止与时间戳传递方式，不用浮点秒重建纳秒时间。

现有浏览器读取后端返回的设备类型，并使用统一 XYZ 回放协议，因此本次无需修改两个上位机目录中的 HTML、CSS 和 JavaScript。

## 4. 测试与验收

### 自动化测试

- 调整 [test_inspection_bridge.py](/home/fire/Desktop/project_east_eleair/ros2_ws/src/functions/communication/agv_bridge/test/test_inspection_bridge.py) 中的原始雷达发布器为 CustomMsg；将雷达加入有效设备断言，检查类型、有效帧、无效帧、重复/倒退时间戳及超时状态。
- 同步调整 [test_websocket_console.py](/home/fire/Desktop/project_east_eleair/tests/system/test_websocket_console.py) 中复用的雷达发布器，验证页面继续接收正确设备状态。
- 在预检测试中覆盖：有效 CustomMsg 更新接收时间；空帧、数量不匹配、无效时间不更新；断流后预检失败。
- 在 [test_replay.py](/home/fire/Desktop/project_east_eleair/tests/operations/test_replay.py) 中覆盖新 CustomMsg、历史 PointCloud2、未知类型、非有限坐标及 5000 点限制。
- 在 [test_validators.py](/home/fire/Desktop/project_east_eleair/ros2_ws/src/functions/data/inspection_tools/test/test_validators.py) 中补充 CustomMsg 校验用例，保留已有 PointCloud2 测试。
- 在隔离 ROS 域、无硬件的合成数据测试中，完成一次真实 MCAP 录制、正常停止、读取与回放；核对完整原始字段，而非仅检查文件存在。

### RDK 实机验收

1. 实际驱动参数 `xfer_format` 为 `1`，`/livox/lidar` 类型为 `livox_ros_driver2/msg/CustomMsg`。
2. 雷达有效数据到达后设备状态正常，断流或持续无效后状态与预检正确变化。
3. 网页能够完成采集预检、开始录制、停止与封存；MCAP 中雷达消息数量大于零，类型正确。
4. 录制前后的 `timebase`、header、点数和逐点字段一致，没有转换或丢弃原始点。
5. 新录包雷达预览正常，历史 PointCloud2 录包仍可回放。
6. 相机、涡流、底盘和位姿功能通过回归检查；未启用雷达的原有采集模式可正常启动。

## 5. 构建与交付

执行 agent 应使用项目现有构建入口，重新构建：

```text
livox_ros_driver2
inspection_bringup
agv_bridge
inspection_operations
inspection_tools
```

RDK 示例命令如下，工程目录以实际部署路径为准：

```bash
cd /media/mcap_data/project_east_eleair_refactor
bash "$PWD/deploy/build.sh" --platform s100p --packages-select \
  livox_ros_driver2 inspection_bringup agv_bridge inspection_operations inspection_tools
source "$PWD/deploy/env.sh" s100p
bash "$PWD/system_websocket/run_robot.sh" --real
```

另一个终端加载同一工程环境后检查实际参数与类型：

```bash
source /media/mcap_data/project_east_eleair_refactor/deploy/env.sh s100p
export ROS_DOMAIN_ID=77
ros2 param get /livox_lidar_publisher xfer_format
ros2 topic type /livox/lidar
```

最终交付修改清单、测试结果和 RDK 构建启动命令，并明确区分已验证项目与待实机验证项目。不能把源码修改完成或页面连接成功当作实机采集验收通过。
