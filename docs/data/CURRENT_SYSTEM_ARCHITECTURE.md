# Physic BEV 当前系统架构

## 1. 系统定位

`physic_bev` 是一套基于 ROS 2 Humble 的多传感器采集、时间关联和 MCAP
数据管理系统。当前阶段重点是可靠采集相机、激光雷达、IMU、涡流、超声波和
机器人状态等原始数据，并生成跨传感器时间索引；目前尚未在在线节点中直接生成
最终 BEV 栅格图。

## 2. 运行环境

| 项目 | 当前配置 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| CPU 架构 | ARM64 / AArch64 |
| ROS | ROS 2 Humble |
| 构建系统 | colcon + ament |
| MCAP | `rosbag2_storage_mcap` |
| 相机 | MindVision MV-SUF1205GM，USB 3.0，4096×3072 Mono8 |
| 激光雷达 | Livox MID-360 |
| 相机 SDK | MindVision Linux SDK ARM64 |
| 雷达 SDK | Livox-SDK2 ARM64 |
| 数据盘 | 板载 eMMC `/dev/mmcblk0` |
| 推荐相机录制频率 | 10 Hz |
| rosbag2 缓存 | 1 GiB |
| MCAP chunk | 64 MiB |

## 3. 总体数据流

```text
MindVision 相机
    │ MindVision SDK
    ▼
mindvision_camera_driver
    ├── /camera/camera/depth/image_rect_raw
    └── /camera/camera/depth/camera_info
                         │
Livox MID-360            │
    │ Livox-SDK2         │
    ▼                    │
livox_ros_driver2        │
    ├── /livox/lidar     │
    └── /livox/imu_raw   │
              │          │
              ▼          │
inspection_livox_adapter │
    └── /livox/imu       │
                         │
涡流 STM32               │
    ▼                    │
eddy_driver              │
    └── /inspection/eddy_current/raw
                         │
机器人/超声波外部节点      │
    ├── /robot/odometry
    ├── /robot/state
    ├── /inspection/ultrasound/raw
    └── /inspection/probe_state
                         │
                         ▼
                  inspection_sync
                         │
                         └── /derived/inspection/fusion_index

原始话题、FusionIndex、TF、诊断信息
                         │
                         ▼
             inspection_bringup / rosbag2
                         │
                         ▼
                       MCAP
                         │
                         ▼
                  inspection_tools
        校验、容量估算、固化、哈希、复制、回放检查
```

## 4. 功能包说明

### 4.1 `mindvision_camera_driver`

MindVision 工业相机 ROS 2 驱动，用于替代原 RealSense D405 启动逻辑。

主要功能：

- 动态加载 MindVision 官方 Python SDK 和 ARM64 `libMVSDK.so`；
- 按序列号选择相机；
- 设置分辨率、曝光、Mono8 输出和连续采集模式；
- 将 MindVision 设备时钟映射到 ROS 时间域；
- 使用直接内存复制，避免 Humble Python 消息逐字节检查造成的性能下降；
- 发布标准 `sensor_msgs/msg/Image` 和 `sensor_msgs/msg/CameraInfo`；
- 支持限制发布频率，硬件录制档默认使用 10 Hz；
- 支持 Ctrl-C 安全释放相机和 SDK。

输出话题：

```text
/camera/camera/depth/image_rect_raw  sensor_msgs/msg/Image
/camera/camera/depth/camera_info     sensor_msgs/msg/CameraInfo
```

说明：`depth` 是为了兼容原工程保留的话题路径。当前图像实际是二维 `mono8`
灰度图，并不是深度图。

当前状态：

- 分辨率：4096×3072 Mono8；
- 不限速时约 24～25 FPS；
- MCAP 录制默认 10 Hz；
- Image 与 CameraInfo 一一对应；
- 相机内参尚未标定，CameraInfo 的 `K/D/P` 当前为零。

### 4.2 `livox_ros_driver2`

Livox 官方 ROS 2 驱动，本工程使用 PointCloud2 输出模式。

主要功能：

- 通过 Livox-SDK2 连接 MID-360；
- 读取 MID-360 JSON 配置；
- 接收并组装点云；
- 发布 MID-360 原始 IMU；
- 支持点云发布频率配置；
- 已修复 ARM64 环境退出时的静态单例析构问题；
- Ctrl-C 时安全移除 SDK observer、停止线程并卸载 SDK。

输出话题：

```text
/livox/lidar    sensor_msgs/msg/PointCloud2
/livox/imu_raw  sensor_msgs/msg/Imu
```

当前设置：

- 点云格式：PointCloud2 / PointXYZRTLT；
- 点云目标频率：10 Hz；
- Frame ID：`livox_frame`。

### 4.3 `inspection_livox_adapter`

MID-360 IMU 单位适配节点。

主要功能：

- 订阅 Livox 原始 IMU；
- 将线加速度从 `g` 转换为 ROS 标准单位 `m/s²`；
- 保留角速度、时间戳、协方差等字段；
- 检查零时间戳、非有限数值和时间倒退；
- 发布转换统计和异常诊断。

数据流：

```text
/livox/imu_raw [g]
        │ × 9.80665
        ▼
/livox/imu [m/s²]
```

输出话题：

```text
/livox/imu   sensor_msgs/msg/Imu
/diagnostics diagnostic_msgs/msg/DiagnosticArray
```

### 4.4 `eddy_driver`

STM32 USB 串口涡流采集驱动。

主要功能：

- 通过 udev 稳定设备链接 `/dev/eddy` 连接；
- 默认串口速率 4 Mbps；
- 解析多通道涡流数据；
- 发布原始 ADC 数据和设备状态；
- 将设备数据转换成统一的 `EddyCurrentFrame`；
- 支持向 STM32 发送控制命令；
- 记录传感器编号、标定编号、增益、采样率和提离量；
- 发布通信诊断。

主要话题：

```text
/inspection/eddy_current/raw  inspection_interfaces/msg/EddyCurrentFrame
/eddy/state                   eddy_driver/msg/EddyState
/eddy/cmd                     std_msgs/msg/Int32MultiArray
~/adc_data                    std_msgs/msg/UInt8MultiArray
/diagnostics                  diagnostic_msgs/msg/DiagnosticArray
```

该功能在默认硬件启动中为可选项，通过 `start_eddy:=true` 启用。

### 4.5 `inspection_sync`

多传感器异步时间关联节点。该节点不复制图像、点云或波形数据，而是生成轻量的
`FusionIndex`，指出一次检测事件应该关联哪些原始消息。

输入话题：

```text
/camera/camera/depth/image_rect_raw
/livox/lidar
/livox/imu
/robot/odometry
/robot/state
/inspection/ultrasound/raw
/inspection/eddy_current/raw
/inspection/probe_state
```

主要功能：

- 为每种传感器维护有界时间环形缓存；
- 使用超声波或涡流消息作为同步锚点；
- 查找最接近锚点的相机帧和点云；
- 提取锚点前后的 IMU 时间窗口；
- 查找锚点前后的机器人位姿并进行插值；
- 检查各数据源的时间误差是否在容差范围内；
- 支持固定软件时钟偏移；
- 提供实验性的自适应时钟偏移估计；
- 记录资产、板材、焊缝和扫描路径标识；
- 对无效关联给出原因和质量标志。

输出话题：

```text
/derived/inspection/fusion_index  inspection_interfaces/msg/FusionIndex
/diagnostics                      diagnostic_msgs/msg/DiagnosticArray
```

`FusionIndex` 只保存时间关联和元数据，原始大数据仍保存在原始话题中。

### 4.6 `inspection_interfaces`

工程统一消息接口包。

| 消息 | 用途 |
| --- | --- |
| `EddyCurrentFrame` | 涡流 I/Q 波形、采样参数、提离量和质量信息 |
| `UltrasoundFrame` | 多通道超声采样、声速、耦合度和接触力 |
| `ProbeState` | 探头接触力、提离量、温度和耦合状态 |
| `RobotState` | 速度、电池、吸附电流、打滑率和扫描状态 |
| `FusionIndex` | 相机、雷达、IMU、位姿与检测锚点的时间关联 |

设计原则：原始大数据保留在各自话题中，`FusionIndex` 只记录如何查找和组合这些
数据。

### 4.7 `inspection_bringup`

系统启动、配置和 MCAP 录制编排包。

主要职责：

- 启动 MindVision；
- 启动 MID-360；
- 启动 IMU 单位适配器；
- 可选启动涡流驱动和同步节点；
- 发布静态 TF；
- 加载 QoS 配置；
- 启动 rosbag2 MCAP 录制；
- 管理调试档和生产档分片参数。

启动文件：

| Launch 文件 | 功能 |
| --- | --- |
| `hardware_pipeline.launch.py` | 完整真实硬件采集入口 |
| `record_only.launch.py` | 只录制已有 ROS 话题 |
| `mock_pipeline.launch.py` | 使用模拟传感器测试整个链路 |
| `baseline_capture.launch.py` | 定时采集相机、雷达、IMU 基线包 |
| `baseline_replay.launch.py` | 回放基线数据 |
| `eddy_sync_capture.launch.py` | 以涡流数据为锚点进行同步采集 |
| `eddy_sync_replay.launch.py` | 回放涡流同步数据 |
| `rough_calibration_view.launch.py` | 查看相机和雷达粗外参效果 |

主要 TF：

```text
camera_link
    └── mindvision_optical_frame

inspection_origin
    └── livox_frame
```

`publish_rough_extrinsics:=true` 时还会发布 MID-360 到相机的粗略外参。粗外参明确
标记为非生产标定结果。

### 4.8 `inspection_mock_sensors`

确定性模拟传感器节点，用于没有真实硬件时测试系统。

模拟发布：

- 灰度/深度兼容图像和 CameraInfo；
- MID-360 PointCloud2 和 IMU；
- 机器人里程计和机器人状态；
- 超声波帧、涡流帧和探头状态；
- TF。

模拟节点支持固定随机种子，因此结果可重复，适用于 CI、MCAP 录制、同步算法、
回放工具和无硬件开发测试。

### 4.9 `inspection_tools`

MCAP 数据工具包。

| 命令 | 功能 |
| --- | --- |
| `validate_bag` | 检查必需话题、消息数量、时间戳和数据格式 |
| `finalize_bag` | 固化采集包并生成不可变交付结果 |
| `estimate_storage` | 估算指定配置的存储需求 |
| `depth_to_pointcloud` | 从兼容深度图和 CameraInfo 重建点云 |
| `inspect_latest` | 查找并检查最新采集包 |

该包还负责 MCAP 哈希、数据复制、元数据整理和采集包完整性验证。

`depth_to_pointcloud` 属于原 D405 流程遗留工具。当前 MindVision 输出为 Mono8，
不能直接用于深度点云重建。

## 5. MCAP 录制内容

当前配置会尝试录制以下话题：

```text
/camera/camera/depth/image_rect_raw
/camera/camera/depth/camera_info
/livox/lidar
/livox/imu_raw
/livox/imu
/robot/odometry
/robot/state
/inspection/ultrasound/raw
/inspection/eddy_current/raw
/inspection/probe_state
/tf
/tf_static
/diagnostics
/derived/inspection/fusion_index
```

只有实际存在发布者的话题才会写入 MCAP。

### 调试档

- 最大分片：2 GiB；
- 最大分片时长：30 秒；
- rosbag2 缓存：1 GiB。

### 生产档

- 最大分片：10 GiB；
- 最大分片时长：900 秒；
- rosbag2 缓存：1 GiB。

## 6. 当前验证结果

MindVision 单独录制结果：

- 时长：41.929 秒；
- Image：409 条；
- CameraInfo：409 条；
- 实际频率约 9.75 Hz；
- 没有 rosbag2 缓存丢帧。

MindVision 与 MID-360 联合录制结果：

- 时长：18 秒；
- MindVision Image：180 条；
- CameraInfo：180 条；
- MID-360 点云：157 条；
- 原始 IMU：3137 条；
- SI 单位 IMU：3141 条；
- diagnostics：15 条；
- MCAP 大小：1.6 GiB；
- 没有 rosbag2 缓存丢帧。

## 7. 当前完成状态

| 模块 | 状态 |
| --- | --- |
| MindVision SDK ARM64 | 已安装并验证 |
| MindVision ROS 2 驱动 | 已完成 |
| 相机 10 Hz 无丢帧 MCAP | 已验证 |
| Livox-SDK2 ARM64 | 已安装 |
| MID-360 ROS 2 驱动 | 已验证 |
| Livox IMU SI 单位转换 | 已验证 |
| MindVision + MID-360 联合 MCAP | 已验证 |
| Livox Ctrl-C 安全退出 | 已修复并验证 |
| TF 基础关系 | 已建立 |
| 涡流驱动 | 已实现，待真实设备联合验证 |
| 时间同步/FusionIndex | 已实现，待完整硬件验证 |
| 机器人状态输入 | 接口已定义，等待外部节点 |
| 超声波输入 | 接口已定义，等待外部节点 |
| 相机内参标定 | 尚未完成 |
| 相机—雷达精确外参 | 尚未完成 |
| 最终物理 BEV 生成 | 尚未接入在线流水线 |

## 8. 常用启动命令

MindVision 与 MID-360 联合录制：

```bash
cd ~/east_eleair/physic_bev
source install/setup.bash
export MINDVISION_SDK_ROOT=~/east_eleair/MindVisionlinuxSDK

ros2 launch inspection_bringup hardware_pipeline.launch.py \
  start_mindvision:=true \
  mindvision_publish_rate_hz:=10.0 \
  start_mid360:=true \
  start_eddy:=false \
  start_sync:=false \
  record:=true \
  publish_rough_extrinsics:=false \
  debug_mode:=true \
  output_directory:=$HOME/east_eleair/physic_bev/bags/physic_bev_$(date +%Y%m%d_%H%M%S)
```

采集结束后检查 MCAP：

```bash
LATEST_BAG=$(ls -dt "$HOME"/east_eleair/physic_bev/bags/physic_bev_* | head -1)
ros2 bag info "$LATEST_BAG"
```
