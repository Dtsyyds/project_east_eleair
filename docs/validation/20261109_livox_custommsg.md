# 上位机 Livox CustomMsg 适配记录

## 交付内容

当前 `system_websocket/run_robot.sh` 的 `all` 模式使用 `config/communication/robot_all.yaml`：启用 MID360，保留 `start_sync=false`。驱动启动参数已改为 `xfer_format=1`，原始 `/livox/lidar` 类型为 `livox_ros_driver2/msg/CustomMsg`。

- `agv_bridge` 的原始雷达订阅、消息状态、缓存与实时预览改为 CustomMsg。结构错误、无效时间戳、重复/倒退时间戳及超时均报告失败。实时预览最多 5 Hz、每帧 2000 点，过滤抽样点中的非有限 XYZ。
- `inspection_operations` 仅在 CustomMsg 的点数、timebase、header 时间有效时更新雷达预检接收时间；持续无效消息不能维持采集就绪。
- `inspection_tools` 新增结构校验，录包检查按 MCAP 中实际记录的类型区分 CustomMsg、历史 PointCloud2 和不支持类型。
- 回放按实际消息类选择解析器，CustomMsg 预览最多 5000 点。header 纳秒时间与 timebase 通过十进制字符串传递，避免浏览器浮点整数精度损失。
- 原始录制流程保持完整 ROS 消息，不抽样、不转换；点级 NaN/Inf 只在显示阶段过滤。保留所有点的 offset_time、XYZ、reflectivity、tag、line，以及帧头、timebase、lidar_id、rsvd。
- 补齐三个消费包对 livox_ros_driver2 的依赖。调整 CMake 依赖发现顺序，解决 Ubuntu 22.04 的 JsonCpp 与 Livox 所引入 PCL/VTK 的重复目标问题。

两套现有网页保持原布局和 R6 磁吸附车模；当前正式入口仍使用 `apps/web_console_chassis_tracking`。D405 已切换的新模型、设备网络 JSON、端口、IMU、TF 和算法派生 PointCloud2 话题均保留。用户已有的 MID360 网络配置更改未被覆盖。

## 构建与安装

本机当前有效安装目录为 `ros2_ws/install`，`deploy/env.sh` 会选择该目录。使用现有 colcon 构建链将五个相关包安装到此处，构建缓存独立放在 `.runtime/s100p/custommsg_build`。通用 `deploy/build.sh` 所需的离线 wheel 目录在当前机器缺失，且其输出路径不同，因此没有切换运行环境或安装新 Python 依赖。

```bash
cd /media/mcap_data/project_east_eleair_refactor
source deploy/env.sh
colcon --log-base .runtime/s100p/custommsg_build_logs build \
  --base-paths ros2_ws/src \
  --build-base .runtime/s100p/custommsg_build \
  --install-base ros2_ws/install \
  --packages-select livox_ros_driver2 inspection_tools inspection_operations inspection_bringup agv_bridge \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

已核对 installed operations/validators 的导入路径和安装后的 launch 参数，实际运行会加载更新后的文件。构建日志在 `.runtime/s100p/analysis/custommsg_20261109/`。

## 验证

证据目录：`.runtime/s100p/analysis/custommsg_20261109/`。

- Python 校验、采集预检、回放、现有采集操作及 MCAP 测试：45 项通过，见 `python_tests.xml`。
- 隔离 ROS 域中运行真实 `ros2 bag record -s mcap`，发布 8 帧、每帧 6001 点的 CustomMsg，正常 SIGINT 停止并读取 MCAP。逐帧比较所有消息字段及每个点的所有字段，包括 NaN/Inf、负零和超过 2^53 的整数时间。全部一致。CDR 未定义的对齐填充字节不作为数据字段比较。
- 同一真实录包通过完整性检查和 Playback 回放；另测历史 PointCloud2 MCAP、未知消息类型、损坏 CustomMsg。结果见 `mcap_verified/test_ros_recorder_preserves_ev0/roundtrip_report.json`。
- 实际安装的 C++ 桥接进程测试 8 项通过、Firefox 页面测试 6 项通过，见 `bridge_tests.xml`、`browser_tests.xml`。`browser_tests/test_custommsg_reaches_r6_cons0/live_custommsg.png` 是合成 CustomMsg 经 ROS、C++ 桥接、WebSocket 送入正式 R6 网页的真实截图，相邻 JSON 记录源点数、显示点数和设备类型。

现有浏览器与桥接整合回归另有 2 项通过，见 `console_regression.xml`；累计 61 项通过。

测试使用 ROS_DOMAIN_ID 187/188/189 与 ROS_LOCALHOST_ONLY=1。合成数据测试不代表实物雷达或底盘验收。

## 现场启动与剩余验收

检查时 `eth0` 配置为 `192.168.2.100/24`，但 `carrier=0`、接口 DOWN；MID360 配置目标为 `192.168.2.3`。当前不能确认实物雷达出点。本次未启动整套硬件服务，测试进程退出后已关闭。

接通雷达供电、网线，确认 eth0 链路恢复后，从当前工程启动正式入口：

```bash
bash system_websocket/run_robot.sh --camera-mode all
```

访问原上位机 `http://192.168.1.148:8080` 或 `http://192.168.127.10:8080`，刷新页面，在实时巡检的雷达点云窗口观察点云和 R6 车模。不要用仅 D405 / cameras 模式验收雷达，这两种模式主动关闭 MID360。

同一运行域（默认 77）确认：

```bash
source deploy/env.sh
export ROS_DOMAIN_ID=77
ros2 param get /livox_lidar_publisher xfer_format
ros2 topic type /livox/lidar
ros2 topic hz /livox/lidar
```

预期参数为 1、类型为 CustomMsg、有持续帧率、页面雷达状态 verified。随后在真实设备上完成采集预检、短时录制/正常停止、录包检查和回放；确认断流后状态与预检变为失败。这些实物项目尚待现场完成。
