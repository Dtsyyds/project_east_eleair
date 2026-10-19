# 罐体机器人二维视觉检测 MVP

这是面向磁吸附罐体巡检机器人的 CPU 可运行基线。当前输出焊缝中心线、像素横向偏差、航向偏差、焊缝丢失状态、图像质量和保守的 `surface_anomaly` 候选区域。项目没有模型权重，也不会在缺少标定时输出毫米或虚构三维坐标。

平台优先级固定为：x86_64 本机开发与测试 → RDK S100P 正式部署 → RK3588 兼容迁移。当前真实执行环境只有 Ubuntu 24.04/x86_64/Python 3.13/ROS 2 Jazzy/OpenCV CPU；没有迈德威视 SDK、S100P、RK3588、BPU/RKNN Runtime 或真实数据。因此本文不会把 fake adapter 单元测试写成硬件推理，也没有模型精度、毫米宽度、凹坑深度或激光三维精度结论。

## 三通路架构

`VisionPipeline` 仍是原有兼容入口，用于确定性离线处理：每个有效输入都按稳定顺序输出，不主动丢帧。实时部署使用三个独立组件：

```text
FrameData（保留采集 timestamp/frame_id）
├─ RealtimeSeamPipeline [默认队列1, drop_oldest]
│  └─ quality + ROI/缩略图 + seam + pixel geometry
├─ InspectionPipeline [默认队列2, 独立触发]
│  └─ 1024/128 tiling + defect inference + NMS + 原图坐标
└─ AsyncResultWriter [有界队列]
   └─ overlay/mask/JSON/CSV/raw 存储策略 + flush/close/低磁盘保护
```

- 实时通路只输出 `SeamResult`、`QualityResult` 和阶段指标，不做缺陷分块或写盘；输入高于处理能力时丢弃排队中的旧帧，绝不修改采集时间戳。
- 精细通路支持 always、帧间隔、编码器距离和外部事件触发。tile 记录 `x/y/valid_width/valid_height`，结果恢复到4096×3072原图坐标。
- 记录通路把编码与磁盘写入移到后台线程。写盘异常形成 warning/failed counter，不终止算法线程；队列支持 block/drop_oldest/drop_newest。
- ROS节点对每帧同步执行低延迟循迹并立即发布 seam/quality；精细检查在独立线程触发，完成后再发布 defects。

`StageTiming` 使用 `time.perf_counter_ns()`，包含 source read、decode、undistort、quality、ROI、preprocess、seam detect/geometry、tiling、defect inference、postprocess、overlay、serialization、writer queue、disk write、algorithm total 和 end-to-end。`PipelineStats` 汇总 mean/P50/P90/P95/max、处理/跳过/丢弃/失败数、最大队列深度和FPS。纯算法时间不包含overlay、序列化或写盘；当前实测见 [TEST_VALIDATION.md](TEST_VALIDATION.md)。

## 跨平台与可选推理后端

运行 `python3 scripts/detect_platform.py` 会综合机器架构、device-tree/board信息、OS/Python/ROS环境以及模块和相机SDK文件探测；`TANK_VISION_PLATFORM` 或 `--platform` 只作显式覆盖，架构不符会报告原因。

| 平台 | 配置 | 首选后端 | 本轮状态 |
|---|---|---|---|
| x86_64 / Jazzy | `configs/platforms/x86.yaml` | ONNX Runtime CPU，缺失时OpenCV CPU | OpenCV路径已实测；本机未安装ONNX Runtime |
| RDK S100P / Humble | `configs/platforms/s100p.yaml` | `DRoboticsBpuBackend`，Nash-m | 可选导入、元数据校验、fake adapter单测；未做板端推理 |
| RK3588 / Humble | `configs/platforms/rk3588.yaml` | `RknnBackend` | 可选导入和安全不可用；未做NPU推理 |

统一 `InferenceBackend` 提供 name、available、unavailable_reason、load_model、infer、warmup、close、input/output spec。ONNX只返回原始张量并检查shape/dtype；没有模型解码元数据时不猜测YOLO输出。S100P模型必须声明 `target_arch: nash_m`，RKNN模型必须声明 `target_arch: rk3588`，配置校验会拒绝跨板卡混用。

## Python与依赖

包声明支持 Python 3.10～3.13（`>=3.10,<3.14`）。依赖按用途拆分：

- `requirements/core.txt`：NumPy、OpenCV、PyYAML；
- `requirements/dev.txt`：本机测试和tox；
- `requirements/x86.txt`：可选ONNX Runtime CPU；
- `requirements/s100p.txt`、`requirements/rk3588.txt`：只保留官方SDK安装说明，不从PyPI误装板端运行时。

本轮只在Python 3.13实测。`tox.ini` 提供 py310/311/312/313 矩阵并跳过本机不存在的解释器；Ubuntu 22.04/Python 3.10/ROS 2 Humble可用 `docker/Dockerfile.humble` 验证，本轮没有实际运行该容器。

## 存储策略

默认不持续保存全部4096×3072原图。`storage` 配置支持：正常帧按 `normal_frame_interval` 降频压缩保存；异常帧保存完整原图及压缩的前后窗口；`save_all_raw` 必须显式开启。预事件缓冲位于异步记录线程并保存压缩字节，避免同步阻塞循迹。磁盘剩余空间低于阈值时停止接收的记录任务并告警，不自动删除任何文件。

## 数据集与训练准备

`dataset/` 只提交Schema、标签和空目录；真实图像/标注/切分输出被忽略。每条样本记录session/tank/scan分组、时间戳、相机、曝光/增益、尺寸、标定版本、质量标记和MCAP位置。`scripts/validate_dataset.py` 检查路径、标签、时间戳、图像尺寸和标定版本；`scripts/create_grouped_split.py` 保证 `(session_id, tank_id, scan_id)` 不泄漏到多个集合。

`training/` 只定义独立的焊缝分割与表面缺陷分割接口、显式exporter入口和ONNX原始张量检查；没有权重、下载数据或训练结果。详细路线见 [ROADMAP.md](ROADMAP.md)。

## 当前能力与边界

- 输入：图片文件夹、视频文件、OpenCV 摄像头；保留来源侧采集时间戳。
- 焊缝：畸变校正开关、ROI、缩略图、通道选择、CLAHE、滤波、局部对比/形态梯度、阈值、形态学、连通域、逐行中心、RANSAC 一次/二次拟合。
- 几何：默认在配置参考行计算 `焊缝 x - ROI 目标中心 x`；相对图像竖直行进方向的角度为航向误差。结果仅为像素和角度。
- 置信度：综合中心点覆盖、拟合内点率、残差、连续性、区域对比度和 ROI 位置合理性；质量异常还会降低置信度。
- 异常：颜色、局部对比和纹理候选，只输出 `surface_anomaly`，不武断声称裂纹、凹坑或锈蚀类别。
- 高分辨率：焊缝支持 ROI/缩略图并恢复原图坐标；异常检测默认 1024 tile、128 overlap，边缘补齐并做坐标恢复和 NMS。原始 `FrameData.image` 始终保留。
- ONNX：`ONNXBackend` 可用 CPU Provider 加载并输出原始张量；模型缺失或 Runtime 缺失会生成明确警告。具体模型预处理/解码应由模型适配器实现。RK3588 NPU 只有明确接口，没有伪实现。
- 蓝色激光：实现 450 nm 蓝通道差分、HSV、形态学、逐列灰度重心亚像素中心和 RANSAC 离群点剔除，也保留激光开/关帧相减。三角测量接口要求完整标定，缺失时返回 `uncalibrated`。
- ROS 2：提供 Jazzy 消息包、节点、launch 和参数；所有发布结果携带输入图像 Header，调试图直接复制输入 Header。

合成图指标仅用于软件回归：清晰直线中心误差不超过 2 px、带噪图不超过 5 px。这不代表真实物理测量精度。

## 目录

```text
tank_vision_inspection/
├── configs/                 # 默认、相机、三平台和 ROS topic YAML
├── dataset/                 # 规范/标签；无真实数据
├── requirements/            # core/dev/x86/S100P/RK3588拆分
├── training/                # 未训练的模型接口和文档
├── tank_vision/
│   ├── sources/             # 文件夹、视频、OpenCV、厂商接口
│   ├── calibration/         # 相机标定与去畸变
│   ├── preprocessing/       # 质量、ROI、坐标变换、归一化
│   ├── seam/                # 传统焊缝检测、中心线和几何
│   ├── defects/             # 传统异常、tile、NMS、ONNX 接口
│   ├── laser/               # 激光线与受标定约束的 3D 接口
│   ├── outputs/             # overlay、mask、JSON、CSV
│   ├── cli.py
│   └── pipeline.py
├── scripts/                 # 合成数据、标定、抽帧、demo
├── tests/
└── ros2_ws/src/             # tank_vision_core + interfaces + ROS wrapper
```

## 安装与离线运行

在本目录运行：

```bash
python3 -m pip install -e .
python3 scripts/generate_synthetic_samples.py
python3 -m tank_vision.cli \
  --config configs/default.yaml \
  --input sample_data/synthetic \
  --output output
```

不安装也可直接在本目录执行相同命令。视频输入会自动识别为 `VideoSource`：

```bash
python3 -m tank_vision.cli --config configs/default.yaml --input inspection.mp4 --output output_video
```

OpenCV 摄像头：

```bash
python3 -m tank_vision.cli --config configs/default.yaml --source camera --camera-device 0 --output output_camera
```

CLI 完成后打印处理帧数、成功帧数、焊缝检测率、异常区域数量、平均耗时和输出目录。输出结构为：

```text
output/
├── overlays/
├── masks/
├── json/
├── raw/                     # 仅按storage策略保存
├── summary.csv
└── performance_summary.json
```

JSON 包含原始文件名、采集时间戳、尺寸、焊缝状态/置信度/几何/中心线、异常区域、图像质量、处理时间、完整配置、算法版本和警告。NumPy 标量、数组和元组都会安全转换为标准 JSON 类型。

实时三通路软件演示（这里仍是文件夹输入，不代表工业相机测试）：

```bash
python3 scripts/run_three_path_demo.py \
  --config configs/default.yaml \
  --input sample_data/synthetic \
  --output output_three_path \
  --pace-hz 15
```

该运行器也支持真实 MindVision 源。使用 USB 2.0 临时配置时不设置
`--pace-hz`，会按相机原生速度提交；实时焊缝通路以最新帧为准，缺陷通路按
`inspection.trigger_every_n_frames`（默认 15）触发：

```bash
MINDVISION_SDK_ROOT='/path/to/linuxSDK_V2.1.0.49...' \
python3 scripts/run_three_path_demo.py \
  --config configs/camera_mv_suf1205gm_usb2.yaml \
  --source mindvision \
  --max-frames 300 \
  --output output_mindvision_three_path
```

## 配置与标定

主配置是 `configs/default.yaml`。阈值、ROI、缩略图、tile、overlap、路径和质量参数均在 YAML 中并在加载时校验。`configs/camera_mv_suf1205gc_t.yaml` 是相机静态描述，不包含未经确认的 SDK feature 名称。

相机棋盘格标定：

```bash
python3 scripts/calibrate_camera.py calibration_images camera_calibration.yaml \
  --columns 9 --rows 6 --square-size 20.0
```

`--square-size` 的单位由用户定义并必须与后续外参/激光标定一致。要启用去畸变，将 `pipeline.use_undistort` 设为 `true` 并填写 `camera.calibration_file`。没有相机、镜头、工作距离和安装姿态标定时，所有尺寸只能解释为像素。

## 测试

```bash
python3 -m compileall -q tank_vision scripts tests
python3 -m pytest -m "not camera and not ros2 and not s100p and not rk3588" -q
python3 -m pytest -q
```

测试覆盖配置、文件夹和损坏文件、时间戳、直线/曲线/噪声/低对比/遮挡/无焊缝、偏差符号、航向角、ROI/缩略图坐标、tile 补齐与恢复、JSON、表面异常、空图像、缺失 ONNX、激光亚像素和输出目录。

## ROS 2 Jazzy与Humble准备

核心包与 ROS 包装隔离。`tank_vision_core` 使用 `ament_cmake_python` 安装项目根的唯一核心源码；构建并 `source install/setup.bash` 后可直接导入，不再需要手工拼接PYTHONPATH。当前机器存在Conda Python 3.13，而Jazzy使用系统Python 3.12，因此构建时显式选择 `/usr/bin/python3`：

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch tank_vision_ros vision.launch.py
```

可复现ROS端到端测试会发布一张图像并检查 seam/quality 的输入stamp和frame_id原样传递：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOG_DIR=/tmp/tank_vision_ros_logs
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
TANK_VISION_RUN_ROS2_E2E=1 /usr/bin/python3 -m pytest tests/test_ros2_integration.py -q -s
```

消息常量 `INTERFACE_VERSION=1`，只使用Humble已有的rclpy、sensor_msgs、geometry_msgs和ament接口。Humble验证方式：以项目根为构建上下文运行 `docker build -f docker/Dockerfile.humble .`，或在S100P的TogetheROS.Bot/Humble环境执行同一colcon/pytest命令。本轮只真实构建Jazzy，未声称Humble构建通过。

默认订阅：

- `/vision/image_raw` (`sensor_msgs/Image`)
- `/camera/camera_info` (`sensor_msgs/CameraInfo`)

默认发布：

- `/vision/debug_image` (`sensor_msgs/Image`)
- `/vision/seam_state` (`tank_vision_interfaces/SeamState`)
- `/vision/defects` (`tank_vision_interfaces/DefectArray`)
- `/vision/quality` (`tank_vision_interfaces/VisionQuality`)

参数在 `ros2_ws/src/tank_vision_ros/config/ros_topics.yaml`。若容器的 home 目录只读，可先设置 `ROS_LOG_DIR=/tmp/ros_logs`。

记录 MCAP：

```bash
ros2 bag record -s mcap \
  /vision/image_raw \
  /camera/camera_info \
  /vision/debug_image \
  /vision/seam_state \
  /vision/defects \
  /vision/quality
```

## MindVision SDK 接入

本机没有搜索到迈德威视/MindVision SDK、头文件、动态库或官方示例，因此没有猜测 API 名称。只读探测和安全冒烟入口：

```bash
python3 scripts/probe_mindvision_sdk.py
python3 scripts/test_mindvision_camera.py \
  --config configs/camera_mv_suf1205gm.yaml \
  --sdk-root '/path/to/linuxSDK_V2.1.0.49...' \
  --frames 30
```

实际硬件接入记录（2026-08-20）：主机已枚举 `MindVision SUF1205GM`
（USB ID `f622:0001`）。该型号是单色 GM 变体，因此使用新增的
`configs/camera_mv_suf1205gm.yaml`，不会覆盖原有 GC 彩色模板。设备枚举为 USB3
Vision，但本次检查中的链路速率为 `480M`，不是 SuperSpeed；在验证全分辨率帧率前，
应先将相机和线缆连接到主机的 USB 3.x SuperSpeed 端口。当前官方 Linux SDK 下载需要
迈德威视账户登录，未取得官方安装包前，脚本只能安全探测、不能进行真实采图。

探测脚本检查可能的 `CameraApi.h`、`libMVSDK.so`、当前架构，并在发现库时运行只读 `file`/`ldd` 信息检查。硬件冒烟脚本使用 SDK 附带的 `mvsdk.py`，连续采集有限帧数并输出实际尺寸、Mono/彩色状态、设备时间戳、曝光、模拟增益和 `source_read_ms`；默认不保存图像。它必须在实际主机终端运行，容器或受限执行沙箱即使能枚举 USB 设备也可能无法访问其控制端点。

实时采集源 `MindVisionCameraSource` 同样使用官方 `mvsdk.py`，将相机的 0.1 ms 设备时间戳换算为纳秒，传递曝光与模拟增益（以 dB 表示），并以 `Mono8` 生成二维灰度帧。指定 SDK 根目录后可直接从设备处理有限帧：

```bash
python3 -m tank_vision.cli \
  --config configs/camera_mv_suf1205gm.yaml \
  --source mindvision \
  --mindvision-sdk-root '/path/to/linuxSDK_V2.1.0.49...' \
  --max-frames 10 \
  --output output_mindvision_live
```

当前主机在 USB 2.0 下已验证 `configs/camera_mv_suf1205gm_usb2.yaml`：
1920×1200、30 ms 曝光约为 19 fps。运行时通过环境变量提供 SDK 路径，避免将本机下载目录写入配置：

```bash
MINDVISION_SDK_ROOT='/path/to/linuxSDK_V2.1.0.49...' \
python3 -m tank_vision.cli \
  --config configs/camera_mv_suf1205gm_usb2.yaml \
  --source mindvision \
  --max-frames 10 \
  --output output_mindvision_usb2
```

`camera.width`/`height` 会匹配 SDK 公布的预设分辨率；必要时可在 YAML 中设置 `camera.resolution_index`。目前实时源只支持连续采集触发模式。缺少 SDK 时返回清晰错误，不会生成假帧。接入位置是 `tank_vision/sources/vendor_camera_source.py`：

1. 安装并确认官方 SDK 与 ARM64/RK3588、Ubuntu 24.04 的兼容性；
1. 用 SDK 自带示例验证丢帧、像素格式和 USB3 带宽；
2. 通过真实罐体数据标定镜头、曝光、增益和缺陷检测阈值；
3. 厂商 SDK 库路径、序列号、触发模式等只放 YAML/环境部署配置，不写死进算法。

## 尚未实现

- 没有真实数据和标注，未训练或附带 YOLO、分割、异常检测权重，ONNX 也没有假定任何模型输出格式。
- 没有相机—激光平面—`base_link` 完整标定，未实现余高、宽度、凹坑深度和毫米坐标。
- 尚未把 MID-360、IMU、LIO、编码器、涡流与视觉结果做时空联合；ROS 消息 Header 和采集时间戳为后续同步提供基础。
- 传统视觉阈值只经过合成数据回归，投产前必须用真实罐体、焊缝、锈蚀、反光、运动模糊和不同曝光数据完成参数标定与误检统计。

建议下一阶段先固定镜头、工作距离、照明和运动速度，采集带时间戳的真实 MCAP；完成相机内参/畸变与相机到 `base_link` 外参；建立焊缝与表面异常标注规范和基准集；再选择 ONNX 模型以及 RKNN/其他 RK3588 NPU 运行时，最后进行激光平面标定和不确定度验证。
