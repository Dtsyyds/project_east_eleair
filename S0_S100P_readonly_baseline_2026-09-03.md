# S0 第一轮只读基线检查报告

**检查日期：** 2026-09-03  
**检查范围：** S100P 工程、运行环境、MindVision、MID-360/IMU、底盘代码、标定、时间关联、MCAP、BPU、Rerun、BEV  
**检查性质：** 只读检查；历史结果不等同于当前板端实测。

## 1. 总体结论

当前板端可确认是 D-Robotics RDK S100P，aarch64，Ubuntu 22.04.5，Linux 6.1 PREEMPT_RT。

- ROS 2 Humble 和 `/opt/tros/humble` 已存在，TROS 的 `dnn_node`、`hobot_cv`、`hobot_shm`、`hobot_usb_cam`、`hobot_bev` 已安装。
- MindVision SDK 已存在，ARM64 `libMVSDK.so` 可加载，且与 `/usr/lib/libMVSDK.so` 相同。
- 当前未观察到 MindVision 相机 USB 设备，未发现 `/dev/video*`；本轮没有当前相机连接、USB 协商速度或实时采集证据。
- 工程中存在真实 MindVision SDK 调用链，但本轮未运行采集。
- 当前视觉主路径仍是 CPU 传统视觉算法；BPU 运行库有安装证据，但没有项目业务模型、BPU 适配器和推理验收证据。
- 没有发现有效相机标定、生产级静态外参、FusionIndex 实现、原始 MCAP 或当前板端验证报告。
- Rerun 代码主要是 ROS 数据展示桥接，物理 BEV 尚未实现；`hobot_bev` 包存在不能证明本项目圆柱资产 BEV 已实现。
- `/home/sunrise/Desktop/Duct_Code` 中有底盘、Livox、FAST-LIO 等源代码，但未安装、未运行，也没有进行融合改造。

目前尚不具备直接进入正式 S1 单相机成像实验的充分条件；具备的是“相机重新接入后进行受控 SDK 烟测”的条件。

## 2. 状态说明

- **本轮已确认：** 本轮通过只读证据确认。
- **历史实测通过：** 工程中已有历史记录，但本轮未复测。
- **仅安装/接口：** 只证明安装或接口存在。
- **未验证：** 存在代码或配置，但缺少运行证据。
- **缺失或冲突：** 发现缺失、过时或相互矛盾。

## 3. 基线汇总

| 项目 | 当前状态 | 证据路径或命令 | 验证时间/范围 | 限制 |
|---|---|---|---|---|
| S100P 硬件 | 本轮已确认 | `/proc/device-tree/model`：`D-Robotics RDK S100P V1P0` | 本轮 | 仅确认板型，不代表所有外设接通 |
| 系统 | 本轮已确认 | `/etc/os-release`、`uname -a` | 本轮 | Ubuntu 22.04.5、aarch64、内核 6.1.158-rt58 |
| 内存/磁盘 | 本轮已确认 | `free -h`、`df -h` | 本轮 | 内存约 21 GiB，磁盘剩余约 19 GiB |
| 系统时间 | 本轮已确认 | `date -Is`、RTC 查询 | 本轮 | 当前时间可读；RTC 显示 1970 年 |
| 时间同步 | 未验证 | `timedatectl`、NTP 服务查询受 DBus 限制 | 本轮 | 未能确认 NTP/chrony 已同步 |
| Git 工程 | 本轮已确认 | `/home/sunrise/east_eleair/tank_vision_inspection` | 本轮 | 分支 `main`，HEAD `33e5941`，工作树有修改和未跟踪文件 |
| 底盘工程 Git | 缺失或冲突 | `/home/sunrise/Desktop/Duct_Code` | 本轮 | 未发现有效 Git 根目录 |
| ROS Humble | 本轮已确认 | `/opt/ros/humble`、`ros2 pkg prefix` | 本轮 | `rosbag2_storage_mcap` 前缀为 `/opt/ros/humble` |
| TROS | 本轮已确认 | 独立子 shell 加载 `/opt/tros/humble/setup.bash` | 本轮 | dnn_node 2.6.1、hobot_cv 2.3.7、hobot_shm 0.0.1、hobot_usb_cam 2.3.1、hobot_bev 2.0.4 |
| ROS Jazzy | 缺失或冲突 | `/opt/ros/jazzy` 不存在 | 本轮 | 工程部分文档仍按 Jazzy 编写 |
| 项目 ROS 包 | 未验证/缺失 | `ros2_ws/src` 有 4 个包；完整 overlay 查询不到 | 本轮 | `tank_vision_ros` egg-link 指向旧机器 `/home/dts/visoin_track/...` |
| Python 系统环境 | 本轮已确认 | `/usr/bin/python3`、`pip show` | 本轮 | Python 3.10.12；PyYAML、pytest 低于项目声明最低版本 |
| Rerun 环境 | 仅安装/接口 | `.venv_rerun`、`rerun-sdk 0.36.2` | 本轮 | 虚拟环境使用系统 site-packages；未启动 Rerun |
| BPU 运行库 | 仅安装/接口 | `hbm_runtime`、`/usr/hobot/lib/libdnn.so`、`HB_HBMRuntime.so` | 本轮 | 未做模型加载和推理；导入时有 UCP memory 错误 |
| BPU 业务推理 | 缺失或冲突 | `tank_vision/inference/d_robotics_bpu_backend.py` | 本轮代码核查 | 没有业务模型路径和配置好的 Nash-M 适配器 |
| MindVision SDK | 本轮已确认 | `MindVisionlinuxSDK/lib/arm64/libMVSDK.so`、`demo/python_demo/mvsdk.py` | 本轮 | ARM64 ELF 正确；未调用采集 |
| 当前 MindVision 设备 | 未验证 | `/sys/bus/usb/devices`、`lsusb -t` | 本轮 | 未发现历史记录中的 `f622:0001` |
| `/dev/video*` | 本轮已确认 | `/dev`、`/sys/class/video4linux` | 本轮 | 未发现节点；不能单独证明 SDK 或协议不可用 |
| MindVision 接入代码 | 未验证 | `tank_vision/sources/vendor_camera_source.py` | 本轮代码核查 | 存在真实 SDK 取帧和资源释放逻辑；未运行 |
| 当前相机配置 | 缺失或冲突 | `configs/camera_mv_suf1205gm.yaml` | 本轮 | 目标为 4096×3072、USB3、15 FPS，无当前设备实测 |
| 历史相机采集 | 历史实测通过 | `output_mindvision_live_60/json/*.json` | 2026-08-20 历史产物 | 1920×1200、Mono8、USB2、约 19 FPS；不能代表当前板端 |
| MID-360 驱动 | 仅安装/接口 | `/home/sunrise/Desktop/Duct_Code/livox_ros_driver2` | 本轮代码核查 | 有源代码和配置，未安装/运行 |
| MID-360 点云时间 | 未验证 | `CustomMsg.msg`、`lddc.cpp`、FAST-LIO preprocess | 本轮代码核查 | `offset_time` 单位未在消息定义中说明 |
| IMU 单位 | 缺失或冲突 | `lidar_imu_data_queue.h`、`lddc.cpp` | 本轮代码核查 | 驱动说明加速度为 g，但直接发布，未发现 SI 转换 |
| FAST-LIO | 仅安装/接口 | `/home/sunrise/Desktop/Duct_Code/fast_lio` | 本轮代码核查 | 配置存在，未安装/运行；`time_sync_en:false` |
| 底盘驱动 | 仅安装/接口 | `robot_go_target/src/duct_robot_move.cpp` | 本轮代码核查 | 有串口 115200、编码器和运动控制实现；当前状态未知 |
| CameraInfo | 缺失或冲突 | `ros2_ws/src/tank_vision_ros/.../vision_node.py` | 本轮代码核查 | 只缓存 CameraInfo，算法未实际使用 |
| 相机标定 | 缺失 | `configs/*`、`dataset/calibration` | 本轮 | 配置均为 `calibration_file:null` |
| 静态外参 | 缺失或冲突 | `Duct_Code/top_pkg/config/static_tf.yaml` | 本轮 | 存在零变换和粗略外参，缺少标定来源 |
| FusionIndex | 未验证 | Rerun `fusion_scalars` 适配逻辑 | 本轮 | 只有展示字段转换，没有实际关联算法证据 |
| MCAP | 仅安装/接口 | `rosbag2_storage_mcap 0.15.16`、README 录制命令 | 本轮 | 未发现 `.mcap`、`metadata.yaml` 或 `.db3` |
| CPU 视觉 | 本轮已确认 | `realtime_pipeline.py`、`classical_baseline.py` | 本轮代码核查 | 当前主路径为传统检测和表面异常启发式方法 |
| 业务缺陷模型 | 缺失 | `inspection_pipeline.py` | 本轮代码核查 | ONNX 路径为不可用占位检测器 |
| Rerun | 仅安装/接口 | `inspection_rerun/rerun_bridge_node.py` | 本轮代码核查 | 有展示桥接；未启动 |
| 物理 BEV | 缺失 | Rerun README、bridge 源码 | 本轮 | 没有圆柱资产 BEV 几何投影或配准实现 |

## 4. 关键代码边界

### 4.1 MindVision

[`vendor_camera_source.py`](./tank_vision_inspection/tank_vision/sources/vendor_camera_source.py) 已包含真实 SDK 枚举、初始化、按序列号选择、分辨率和像素格式设置、曝光设置、取帧、设备时间戳、帧号、曝光、增益和资源释放逻辑。

确认的资源释放路径包括：

- `CameraReleaseImageBuffer()`；
- 对齐内存释放；
- `CameraUnInit()`。

SDK 的 `uiTimeStamp` 按 0.1 ms 解释并转换为 ns。代码没有看到显式设置目标 FPS 的调用。

### 4.2 视觉算法

当前入口仍是 CPU 传统路径：图像质量、ROI、去畸变、接缝检测，以及颜色/对比度/纹理启发式表面异常检测。

没有发现已经接入的焊缝/缺陷业务模型。BPU 后端仅具备接口和运行库探测逻辑，模型路径为空，业务适配器未配置。

### 4.3 Rerun、Observation 和 BEV

Rerun bridge 可以展示图像、CameraInfo、点云、IMU、TF、里程计、路径、诊断和可选检测结果。它不能证明：

- 已完成图像—点云—位姿时间关联；
- 已完成真实 CameraInfo 和外参标定；
- 已完成圆柱资产坐标配准；
- 已实现物理 BEV。

当前 `hobot_bev` 的存在只代表系统包已安装。

## 5. 重要冲突与证据缺口

1. [`README.md`](./tank_vision_inspection/README.md) 仍描述 Ubuntu 24.04、x86_64、Python 3.13、ROS Jazzy，并称 SDK 不存在；与当前 S100P 板端实际情况不符。
2. [`TEST_VALIDATION.md`](./tank_vision_inspection/TEST_VALIDATION.md) 使用旧机器路径 `/home/dts/visoin_track/tank_vision_inspection`，其中 Jazzy、x86、无 SDK、无 S100P 等结论不能作为当前板端结论。
3. 相机目标配置为 4096×3072、USB3、15 FPS；历史采集为 1920×1200、Mono8、USB2、约 19 FPS。当前设备未出现，无法确定当前实际配置。
4. `ros2_ws/install/tank_vision_ros` 的 egg-link 指向旧机器路径，当前 ROS overlay 不能视为已验证可运行。
5. 所有相机配置均未提供有效标定文件；CameraInfo 没有闭合到实际标定源。
6. `static_tf.yaml` 有零值变换，FAST-LIO 另有粗略外参；没有生产标定证据。
7. MID-360 自定义点云的 `offset_time` 单位没有在消息定义中明确；IMU 加速度为 g，但没有发现转换为 ROS SI 单位的代码。
8. FusionIndex 只有展示字段适配，未发现实际关联容差、位姿插值、IMU 窗口门控和成功率统计实现。
9. Rerun 默认图像、里程计和路径话题与当前项目及 Duct 代码话题不一致。
10. 当前没有发现原始 MCAP 或 metadata；历史哈希、回放成功和无丢帧说明均未在本轮复测。

## 6. Git 改动分组建议

当前仓库为 `main`，HEAD 为 `33e5941`，工作树有修改和未跟踪文件。

建议后续按以下边界整理，暂不提交：

1. MindVision 接入：相机源、相机配置、SDK 烟测脚本和对应测试。
2. Rerun/S100P 展示：`ros2_ws/src/inspection_rerun/**` 和相关 setup 脚本。
3. CLI、跨平台配置、测试和文档。
4. `camera_test/mindvision_test.bmp` 作为原始采集证据单独处理，不与功能代码混入同一提交。

本轮没有执行 `git add`、commit、stash、reset、checkout、clean、pull。

## 7. 可复用命令

以下命令是后续候选命令，本轮没有执行会访问硬件或启动节点的部分。

### MindVision 30 帧烟测

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection

/usr/bin/python3 scripts/test_mindvision_camera.py \
  --config configs/camera_mv_suf1205gm_usb2.yaml \
  --sdk-root /home/sunrise/east_eleair/MindVisionlinuxSDK \
  --frames 30 \
  --timeout-ms 1500
```

不传 `--save-image` 时不保存图像，但仍会启动相机采集，因此本轮未执行。

### ROS 环境

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
```

应使用独立子 shell，不加载 Jazzy。

### MCAP 录制

工程 README 中的录制命令为：

```bash
ros2 bag record -s mcap \
  /vision/image_raw \
  /camera/camera_info \
  /vision/debug_image \
  /vision/seam_state \
  /vision/defects \
  /vision/quality
```

该命令明确使用 MCAP，但没有显式配置分卷大小或时长。本轮未录制。

## 8. S1 单相机实验判断

### 已具备

- S100P 板端确认；
- ARM64 MindVision SDK 和系统库确认；
- 工程中存在真实 SDK 取帧和资源释放代码；
- 有历史 1920×1200 Mono8 采集记录；
- 有受控 30 帧烟测脚本。

### 阻塞项

- 当前没有发现 MindVision USB 设备；
- 当前没有实时帧、设备时间戳、帧号连续性和丢帧证据；
- ROS overlay 安装不完整且含旧机器路径；
- CameraInfo、内参、畸变、ROI 和标定版本缺失；
- 目标 USB3/4096×3072 配置尚未得到设备实测；
- 部分 Python 依赖低于项目声明版本。

S1 应限定为单相机、固定视角、无 BPU、无 LIO、无底盘运动、无 Rerun Viewer、无机器人控制。

## 9. 下一轮最多三项操作

1. 重新接入并确认相机 USB VID/PID、序列号和 sysfs 实际速率。
2. 使用历史 USB2 配置执行 30 帧 SDK 烟测，检查分辨率、像素格式、帧号、设备时间戳、曝光、增益、读取耗时和退出释放。
3. 烟测通过后进行固定配置的单相机实验；先以 1920×1200 Mono8 建立可重复基线，再进行有效标定。暂不接入 BPU、LIO、Fusion、Rerun 或底盘。

## 10. 本轮执行范围确认

本轮已执行路径和 `AGENTS.md` 查找、Git 状态检查、源码和文档阅读、系统及 ROS/TROS 环境查询、Python/Rerun/BPU 安装证据检查、MindVision SDK ELF/动态库检查、USB/设备节点/进程只读检查，以及历史产物的有限抽查。

本轮未启动相机、雷达、IMU、录制器、同步节点、Viewer 或机器人程序；未执行采集烟测、ROS 构建/运行、BPU 推理、LIO、底盘串口、MCAP 录制/回放或标定采集；未安装升级依赖。

本轮未修改源码、配置、文档、系统环境、设备状态或原始数据，也未执行任何 Git 写操作。
