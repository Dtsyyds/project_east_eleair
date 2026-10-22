# FIELD-W5 整车多传感器采集系统交接记录

- 日期：2026-09-22
- 时区：Asia/Shanghai
- 工作区：`/home/sunrise/east_eleair`
- 主工程：`/home/sunrise/east_eleair/physic_bev`
- 视觉工程：`/home/sunrise/east_eleair/tank_vision_inspection`
- 正式数据盘：`/dev/nvme0n1p1`，ext4，挂载到 `/media/mcap_data`

## 1. 当前目标与最终决策

本阶段把现有工程整理成整车多传感器采集、分进程写盘、状态监控、停止、校验、归档和统一回放系统。

用户最终确认：正式采集期间不运行焊缝 tracker。采集验收只关注以下数据链路：

- MindVision 工业相机输入和硬件编码视频；
- Intel RealSense D405；
- Livox MID-360 点云和 IMU；
- 轻量状态、诊断、TF、CameraInfo；
- MCAP、视频、时间戳、丢帧、队列、磁盘和停止完整性。

因此 `start_vehicle_collection.sh` 当前默认 `enable_tracker=false`。不开 tracker 时，依赖 tracker 事件的自动失败快照也会自动关闭。tracker 仍保留在离线 `algorithm-replay` 模式中使用。

## 2. 已实现的数据架构

每个 run 位于：

```text
/media/mcap_data/runs/RUN_ID/
├── .partial/                 # 正在写入的 rosbag 目录
├── mcap/
│   ├── core/
│   ├── mid360/
│   ├── d405/
│   └── raw_burst/
├── video/front/
├── failure_snapshots/
├── live_session/             # 复用 failure_event_recorder 时的实际会话目录
├── roi_validation/
├── logs/
├── reports/
├── frozen_configs/
├── run_manifest.json
├── topic_inventory.json
├── device_inventory.json
├── collection_status.json
├── sha256sums.txt
├── field_notes.md
└── complete.json
```

写盘单元相互独立：

1. `core_recorder`：CameraInfo、diagnostics、TF、采集状态；启用 tracker 时才包含 geometry/state。
2. `mid360_recorder`：MID-360 点云、原始 IMU、适配后 IMU及诊断。
3. `d405_recorder`：实际发现的 depth/infra Image、CameraInfo、Metadata、Extrinsics、TF 和诊断。
4. `front_video`：订阅 `/camera/front/image_h264`，写 MKV 和逐帧 JSONL sidecar。
5. `front_video_encoder`：RDK S100P `hobot_codec` 硬件 H.264 编码器，只订阅 ROS 图像，不直接打开 MindVision SDK。
6. `vehicle_collection_monitor`：低频状态、磁盘、CPU、温度、频率、writer 速率和队列统计。
7. `vehicle_collection_disk_guard`：磁盘达到 hard-stop 阈值时调用精确停止流程。

所有进程保存 PID、PGID、`/proc` start ticks 和命令行。停止时只处理当前 run 注册的进程，顺序为 SIGINT、SIGTERM、最后对明确进程组使用 SIGKILL。

## 3. 新增和修改的主要文件

新增入口：

- `scripts/start_vehicle_collection.sh`
- `scripts/status_vehicle_collection.sh`
- `scripts/stop_vehicle_collection.sh`
- `scripts/finalize_vehicle_collection.py`
- `scripts/validate_vehicle_collection.py`
- `scripts/replay_vehicle_collection.sh`
- `scripts/replay_vehicle_collection.py`

新增内部组件：

- `scripts/vehicle_collection_common.py`
- `scripts/vehicle_collection_manager.py`
- `scripts/vehicle_collection_monitor.py`
- `scripts/vehicle_collection_disk_guard.py`
- `scripts/stop_vehicle_collection.py`
- `scripts/front_h264_recorder.py`
- `scripts/front_video_recorder.py`（软件编码后备实现，未选为正式路径）
- `scripts/generate_vehicle_collection_summary.py`

修改的已有文件：

- `src/inspection_bringup/launch/visual_field_validation.launch.py`
  - 增加 `camera_qos_reliability`；
  - 增加 `enable_tracker`；
  - `enable_tracker=false` 时只启动 MindVision 驱动和静态 TF，不启动 `stable_seam_node`。

没有修改 tracker 算法、YAML、ROI、reference、DP、IRLS、gate 或 temporal。没有直接修改 build/install/log 中的生成文件；launch 修改后已经执行：

```bash
colcon build --packages-select inspection_bringup --symlink-install
```

## 4. 实际传感器 topic、类型和 QoS

MindVision：

- `/camera/front/image_raw`：`sensor_msgs/msg/Image`
- `/camera/front/camera_info`：`sensor_msgs/msg/CameraInfo`
- 正式硬件视频模式下 publisher 为 RELIABLE/VOLATILE
- 相机 serial：`046081820650`
- 4096×3072、BGR8、目标约 10 Hz

视频编码输出：

- `/camera/front/image_h264`：`img_msgs/msg/H26XFrame`
- 编码器：`hobot_codec 2.3.5`，S100P VPU H.264
- 容器：MKV
- sidecar：`front_NNNNNN.frames.jsonl`

MID-360：

- `/livox/lidar`：`sensor_msgs/msg/PointCloud2`
- `/livox/imu_raw`：`sensor_msgs/msg/Imu`
- `/livox/imu`：`sensor_msgs/msg/Imu`
- publisher 为 RELIABLE/VOLATILE
- 雷达 IP：`192.168.2.3`
- 主机 `eth0`：`192.168.2.100`

D405 实际发现的流：

- `/d405/camera/depth/image_rect_raw`
- `/d405/camera/infra1/image_rect_raw`
- `/d405/camera/infra2/image_rect_raw`
- 三路对应的 `camera_info` 和 `metadata`
- `depth_to_infra1`、`depth_to_infra2` extrinsics
- D405 静态 TF

D405 没有假设 IMU，也没有假设不存在的 depth-scale topic。图像 publisher 实测为 RELIABLE/TRANSIENT_LOCAL，CameraInfo 为 RELIABLE/VOLATILE。

`/diagnostics` 有多个发布者。离线校验按 `status.name`、`hardware_id` 分组，不能用聚合 topic 的 Header 顺序直接判定回退；MCAP 不保存 publisher GID，因此离线报告会明确标记 GID 不可用。

## 5. 编码器审计与选择

实际测试结果：

- GStreamer 存在，但未发现可用的 `x264enc`、`openh264enc` 或 `v4l2h264enc`。
- FFmpeg `h264_v4l2m2m` 失败：没有有效 V4L2 encoder device。
- FFmpeg `h264_omx` 失败：缺少 OMX 库。
- FFmpeg `libx264` 软件编码可运行，但未选为正式方案。
- RDK `hobot_codec 2.3.5` 硬件 H.264 已在 4096×3072、约 10 Hz 下真实测试，因此被选为正式方案。

正式测试中硬件视频码率约 12～14 Mbit/s。视频 recorder 必须在 encoder 前启动，并等待 DDS 匹配，确保收到初始 SPS/PPS/IDR。

## 6. 存储与带宽证据

SSD preflight 已确认：

- source：`/dev/nvme0n1p1`
- filesystem：ext4
- mount：`/media/mcap_data`
- options：rw,noatime
- 不是根分区或 eMMC

512 MiB direct+fsync 实测约 810～811 MB/s。

旧的全链路 run（包含 tracker 持续 LOST 和 42 个快照）观测到：

- MCAP：约 4.338 GB
- 视频：约 0.245 GB
- live session/快照：约 1.035 GB
- 平均总写入：约 40.34 MB/s
- 简单投影：约 145.23 GB/h

该 145.23 GB/h 包含异常密集快照，不代表新的无 tracker 采集典型值。无 tracker、无快照的长期速率应以新的干净 run 重新测量。

## 7. 已完成的实测

### 7.1 D405 单机

最初使用 `diagnostics_period=1.0` 时出现 UVC control query 和无帧问题。关闭 RealSense 内部周期诊断查询后：

- depth CameraInfo：约 30.020 Hz
- infra1 CameraInfo：约 30.022 Hz
- infra2 CameraInfo：约 30.021 Hz
- Header 回退：0

因此正式启动参数固定为 `diagnostics_period:=0.0`，设备信息和运行状态由静态 inventory 与外部 monitor 记录。

### 7.2 带 tracker 的历史 S1

run：`FIELD-W5-S1-FORMAL-20260922_2242`

传感器和存储链路结果：

- MindVision CameraInfo：9.973 Hz
- MID-360 点云：9.573 Hz
- MID-360 raw IMU：199.996 Hz
- MID-360 adapter IMU：199.998 Hz
- D405 depth image：29.676 Hz
- D405 infra1 image：28.567 Hz
- D405 infra2 image：29.337 Hz
- MindVision timeout：0
- MindVision capture error：0
- 视频：1363 个 sidecar access units、1340 个可解码帧，98.31%
- 视频 queue drop：0
- 视频 write error：0
- MCAP：core、MID-360、D405 均可读
- `/diagnostics` 按源回退：0
- 42 个快照均 `complete=true` 且 hash 通过
- run SHA：330/330 通过
- 停止后无孤儿进程

该 run 当时因 tracker 只有约 0.534 Hz 被判失败。用户后来明确正式采集不运行 tracker，所以该失败条件不再适用于新的 sensor-only run。

### 7.3 无 tracker 采集试运行

run：`FIELD-VEHICLE-20260922_232714`

该 run 已验证：

- `enable_tracker=false` 确实没有启动 `stable_seam_node`；
- MindVision、MID-360、D405、三个 recorder、硬件视频编码器均运行；
- 视频 queue drop=0、write error=0；
- 停止后所有采集进程已结束；
- core、MID-360、D405 已封存到 `mcap/`；
- 两个 MKV 分段已关闭。

但该 run 在运行期间发生过一次旧 run 的 `core-only` 回放测试，同名 CameraInfo、diagnostics 和 TF 被注入当前 ROS domain，造成旧时间戳进入 core recorder。因此该 run 只能作为功能试运行，不能作为正式干净验收数据。

停止时用户在离线 `ros2 bag info` 阶段按了 Ctrl-C。录制进程和封存已经结束，但 `complete.json` 仍为 `pending`。这不是 recorder 未停止。

## 8. 启动、状态与停止命令

正式无 tracker 采集：

```bash
cd /home/sunrise/east_eleair/physic_bev

export FIELD_DATA_ROOT=/media/mcap_data
export FIELD_CAMERA_SERIAL=046081820650
export FIELD_RUN_ID="FIELD-VEHICLE-$(date +%Y%m%d_%H%M%S)"

scripts/start_vehicle_collection.sh \
  run_id:="$FIELD_RUN_ID" \
  record_core:=true \
  record_mid360:=true \
  record_d405:=true \
  record_front_video:=true \
  enable_tracker:=false \
  auto_save_failures:=false \
  provenance_enabled:=false \
  record_front_raw_burst:=false
```

一次状态输出：

```bash
scripts/status_vehicle_collection.sh "$FIELD_RUN_ID" --once
```

持续状态界面：

```bash
scripts/status_vehicle_collection.sh "$FIELD_RUN_ID"
```

默认停止当前 run：

```bash
scripts/stop_vehicle_collection.sh
```

无参数停止规则：

1. 优先使用 shell 中的 `FIELD_RUN_ID`；
2. 没有该变量时，解析唯一一个仍有注册进程存活的 run；
3. 多个活跃 run 时拒绝猜测，要求显式 RUN_ID；
4. 没有活跃 run 时安全报错。

只停止和封存、不等待离线校验：

```bash
FIELD_SKIP_FINALIZE=1 scripts/stop_vehicle_collection.sh
```

之后单独校验：

```bash
python3 scripts/finalize_vehicle_collection.py \
  "/media/mcap_data/runs/$FIELD_RUN_ID"
```

`ros2 bag info`、逐消息时间戳校验、视频三点解码和 SHA 可能需要数分钟。看到：

```text
recording processes stopped
MCAP writers sealed; starting offline validation
```

第一行即表示实际录制已经停止，后续均为离线处理。

## 9. 回放

统一入口：

```bash
scripts/replay_vehicle_collection.sh \
  RUN_ID \
  {core-only|perception|full-sensor|algorithm-replay} \
  {0.5|1.0} \
  START_OFFSET_SEC \
  DURATION_SEC
```

示例：

```bash
scripts/replay_vehicle_collection.sh \
  FIELD-W5-S1-FORMAL-20260922_2242 \
  full-sensor \
  1.0 \
  0 \
  10
```

回放节点曾因试图写入 `rclpy.Node.publishers` 只读属性而失败，现已改为私有 `_field_publishers`。真实测试已完成：

- 3 秒 `full-sensor` 回放能够发布工业相机、D405、TF 和 core 数据；
- 1 秒 `core-only` 回放完成并生成 `executed=true` 的报告。

已知限制：4096×3072 H.264 目前由 OpenCV 软件解码，S100P 上 full-sensor 回放明显慢于实时。只查看视频时优先使用：

```bash
ffplay /media/mcap_data/runs/RUN_ID/video/front/front_000001.mkv
```

严禁在正式采集使用的同一个 ROS domain 内同时回放。后续应增加启动互斥或为回放强制设置独立 `ROS_DOMAIN_ID`。

## 10. 当前已知问题与下一次优先事项

1. 需要重新执行一个完全干净、无 tracker、无回放干扰的 60 秒 sensor-only S1。
2. S1 通过后再执行 10 分钟和 30 分钟测试。
3. 需要测量无 tracker、无快照条件下的真实每小时容量。
4. full-sensor 回放的 4K 视频软件解码较慢；后续可接入实际测试过的 S100P 硬件解码路径。
5. 采集和回放尚未强制使用不同 ROS domain，必须补互斥保护。
6. `FIELD-VEHICLE-20260922_232714` 已停止，但因回放污染和 finalize 被中断，不能作为正式验收 run。
7. 新增的重复 stop/已封存目录兼容逻辑和无参数 active-run 解析已完成静态检查；active-run 的“无活跃”和“唯一活跃”分支均已测试。
8. CameraInfo/外参只作为原始证据保存；不声明已完成空间标定、硬同步、BEV 或毫米测量。

## 11. 后续接入涡流传感器的建议路径

优先复用现有 `eddy_driver`，先审计实际 launch、节点、topic、消息类型、QoS、设备时钟和诊断，不要预设 topic 名。

建议按以下顺序扩展：

1. 在 `vehicle_collection_manager.py inventory` 中增加实际发现的 `record_groups.eddy`。
2. 在 `start_vehicle_collection.sh` 增加：
   - `record_eddy:=true|false`；
   - eddy driver 启动；
   - 数据实际到达门禁；
   - 独立 `eddy_recorder`，输出到 `.partial/eddy`。
3. 停止时将 `eddy_recorder` 加入 recorder 优先停止序列。
4. `seal` 将 `.partial/eddy` 原子移动到 `mcap/eddy`。
5. monitor 增加涡流输入 Hz、Header/设备时间、队列、drop、write error 和 MB/s。
6. validator 增加消息非零、schema/channel、按源时间戳单调、频率和最大间隔检查。
7. replay 的 `full-sensor` 模式纳入 `mcap/eddy`，仍由单一调度器发布，禁止第二个 `/clock`。
8. preflight 带宽模型加入涡流实际测得数据率，并重新计算 1.5～2 倍 SSD 余量。
9. 先执行涡流单独测试，再执行 MindVision+涡流，然后逐步加入 D405 和 MID-360；首次失败后停止叠加。

低带宽涡流状态和诊断可进入 core，但高频原始波形建议使用独立 `mcap/eddy` recorder，避免影响相机、D405 和 MID-360。

涡流数据应尽可能同时保存：

- 设备原始时间；
- ROS Header 时间；
- host receive wall time；
- host monotonic time；
- 通道号、量程、采样率和设备状态；
- 丢样、溢出、通信错误和驱动队列深度。

在没有硬件触发/同步证据前，只能声明 transport timestamp consistency，不能声明涡流与相机、D405、MID-360 已硬同步或空间对齐。

## 12. 证据位置

源码与运行环境审计：

```text
/home/sunrise/east_eleair/output/field_w5_vehicle_collection/
```

最终汇总：

```text
/home/sunrise/east_eleair/tank_vision_inspection/output/field_w5_vehicle_collection/
```

主要报告包括：

- `source_inventory.json`
- `topic_inventory.json`
- `encoder_inventory.json`
- `storage_bandwidth_report.json`
- `performance_matrix.json`
- `timestamp_validation.json`
- `video_validation.json`
- `mcap_validation.json`
- `replay_validation.json`
- `snapshot_integrity.json`
- `orphan_process_check.json`
- `system_acceptance.json`
- `next_action.json`

历史证据 run：

```text
/media/mcap_data/runs/FIELD-W5-S1-FORMAL-20260922_2242
/media/mcap_data/runs/FIELD-VEHICLE-20260922_232714
```

保留现有 dirty workspace；没有执行 git clean/reset/checkout，也没有创建 Git commit。

## 13. 2026-09-23 回放补充

`FIELD-VEHICLE-20260922_234713` 的 MCAP 数据实际存在且可读。原
`full-sensor` 回放把 4096×3072 MKV 的 OpenCV 软件解码与 MCAP 发布放在
同一个调度线程中；S100P 上解码不能达到实时速度，会使 D405/MID-360 发布
长时间排在视频解码之后。这是回放性能问题，不是传感器漏录。

本次增加了 `sensor-only` 模式，用于回放 core、D405 和 MID-360，不进行
工业相机视频解码；同时所有 bag publisher 会在调度前创建，便于 rviz2 和
`ros2 topic list` 立即发现话题。2 秒实测通过，覆盖：

- `/livox/lidar`
- `/livox/imu`、`/livox/imu_raw`
- `/d405/camera/depth/image_rect_raw`
- `/d405/camera/infra1/image_rect_raw`
- `/d405/camera/infra2/image_rect_raw`
- 对应 CameraInfo、metadata、extrinsics、diagnostics 和 TF

完整传感器数据回放命令（末尾 `0` 表示不限时长）：

```bash
scripts/replay_vehicle_collection.sh \
  FIELD-VEHICLE-20260922_234713 sensor-only 1.0 0 0
```

本 run 的工业相机编码资料为单个 `front_000001.mkv`，共 1338 个可解码帧、
约 136.5 秒。可使用 `ffplay` 独立完整查看。当前不要把 4K 软件解码版
`full-sensor` 当作实时统一可视化入口；后续需要接入并实测 `hobot_codec`
硬件解码后再解除这一限制。

`sensor-only` 首次现场运行暴露出调度等待的微小竞态：循环条件与
`time.sleep()` 参数分别读取 monotonic 时间，越过目标时可能生成极小负值。
现已改为每轮只计算一次 `remaining`，小于等于零时直接退出等待。修复后使用
同一 run 完成 2 秒实测，Livox 点云 9 条、两路 IMU 各 194 条，D405 三路
图像及配套 CameraInfo/metadata 均成功发布，无异常退出。
