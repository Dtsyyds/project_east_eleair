# East Eleair 工程功能包、实现原理与运行指南

> 文档依据：2026-09-14 当前工作区源码静态审阅。本文以源码为准，重点覆盖自研功能包、关键实现文件、类/函数、参数、数据流和可执行命令；`build/`、`install/`、`log/`、`output*/`、数据包、虚拟环境及第三方库内部实现不逐文件展开。

## 1. 工程边界与结论

当前目录不是一个单一程序，而是两套相互关联、但可独立构建的系统：

1. `tank_vision_inspection/`：罐体二维视觉检测。负责焊缝检测/跟踪、表面异常候选、蓝色激光线提取、结果记录、ROS 2 发布和 Rerun 可视化。
2. `physic_bev/`：多传感器采集与时序关联工作区。负责 MindVision、Livox、涡流、超声、机器人状态等数据采集，生成 `FusionIndex`，录制/校验 MCAP，并保留 FAST-LIVO2 建图定位链路。

辅助目录：

- `MindVisionlinuxSDK/`：迈德威视官方 SDK、Python 绑定、动态库和示例，是相机驱动依赖，不是本项目业务源码。
- `third_party/`：Sophus、Livox-SDK2 等第三方依赖。
- `physic_bev/src/realsense-ros/`、`livox_ros_driver2/`、`rpg_vikit/`：上游功能包；本工程主要通过配置、launch 和少量兼容修复使用它们。
- 根目录 `camera_collect.py`、`hello.py`、`ways.txt` 等不是两套主流水线的正式入口。

非常重要：`physic_bev` 当前名称虽然包含 BEV，但在线节点尚未生成最终物理 BEV 栅格；目前完成的是多传感器原始数据采集、索引和可视化接口。

## 2. 总体架构与数据流

### 2.1 二维视觉主链路

```text
文件夹 / 视频 / OpenCV 相机 / MindVision SDK / ROS Image
                         │
                         ▼
                     FrameData
                         │
          ┌──────────────┴──────────────┐
          ▼                             ▼
RealtimeSeamPipeline             InspectionPipeline
图像质量 + 焊缝检测               高分辨率分块 + 异常候选
最新帧语义，低延迟                独立触发、独立线程
          │                             │
          └──────────────┬──────────────┘
                         ▼
                  VisionResult / ROS msg
                         │
              ResultWriter / AsyncResultWriter
                         ▼
             overlay、mask、JSON、CSV、raw
```

离线 `VisionPipeline` 会同步执行两条算法通路，对每个有效输入产生一个有序结果；实时 ROS 节点则同步发布焊缝和质量，异步发布耗时更高的缺陷结果。

### 2.2 多传感器采集链路

```text
MindVision ──> Image/CameraInfo ─┐
MID-360 ────> PointCloud2 ───────┤
MID-360 IMU -> SI单位Imu ────────┤
STM32涡流 ──> EddyCurrentFrame ──┤
超声/机器人/探头外部节点 ────────┤
                                 ▼
                         inspection_sync
                      最近邻、窗口、位姿插值
                                 │
                                 ▼
                            FusionIndex
                                 │
                   inspection_bringup/rosbag2
                                 ▼
                               MCAP
                                 │
                 inspection_tools / inspection_rerun
```

`FusionIndex` 只存时间关联和派生位姿，不复制图像、点云或波形，原始消息仍在原话题和 MCAP 中。

## 3. `tank_vision_inspection` 核心 Python 包

### 3.1 配置系统：`tank_vision/config.py`

关键类与函数：

- `CameraConfig`：尺寸、来源、帧 ID、SDK 根目录、序列号、像素格式、曝光、帧率、触发、超时和相机 ROI。
- `PipelineConfig`：自然焊缝模式、去畸变、算法 ROI、缩略图上限以及输出开关。
- `SeamConfig`：传统焊缝检测的预处理、阈值、形态学、连通域、RANSAC 和置信度参数。
- `DefectConfig`：异常候选分块、局部对比/纹理/颜色阈值、面积、排除区和 NMS 参数。
- `LaserConfig`：450 nm 蓝线的 HSV、通道差分、形态学、最少点数和 RANSAC 参数。
- `RealtimeSeamConfig`、`InspectionConfig`、`StorageConfig`：三通路的队列、丢帧策略、触发策略和写盘策略。
- `InferenceConfig`、`PlatformConfig`：推理后端与平台约束。
- `load_config(path)`：读取 YAML，拒绝未知 section/字段，构造 `AppConfig` 后调用 `validate_config()`。
- `validate_config(config)`：校验比例范围、奇数卷积核、ROI、队列容量、丢帧策略、触发模式、平台/模型架构等。

关键注意事项：

- 配置不是宽松字典：拼错字段会直接报 `Unknown keys`，不会静默忽略。
- 所有形态学/滤波核必须是正奇数。
- `process_latest_only=true` 必须配 `drop_policy=drop_oldest`。
- `overlap` 必须满足 `0 <= overlap < tile_size`。
- 自动曝光要求 `exposure_us=null`；手动曝光要求 `exposure_us>0`。
- S100P BPU 模型目标必须是 `nash_m`；RKNN 不能配置给 S100P。

主配置是 `tank_vision_inspection/configs/default.yaml`。稳定跟踪使用另一套独立 schema：`s100p_stable_seam_4096x3072.yaml` 或 `s100p_guided_anchor_seam_4096x3072.yaml`，不能直接交给 `load_config()`。

### 3.2 数据类型与性能统计

实现文件：

- `tank_vision/types.py`：`FrameData`、`QualityResult`、`SeamResult`、`DefectResult`、`VisionResult`、`RealtimeResult`、`InspectionResult`、`LaserLineResult`、`Profile3DResult`。
- `tank_vision/metrics.py`：`StageTiming`、`StageTimer`、`FrameMetrics`、`PipelineStats`。

`FrameData` 是所有入口的统一数据契约，保留图像、采集时间戳、frame_id、序号、曝光、增益、标定版本和来源元数据。算法不得用处理时刻覆盖采集时间戳。

`StageTimer` 用 `perf_counter_ns()` 记录阶段耗时，`PipelineStats.summary()` 汇总 mean、P50、P90、P95、max、FPS、处理/跳过/丢弃/失败数和最大队列深度。调优时应优先看 `total_algorithm_ms`，不要把写盘时间误算成算法时间。

### 3.3 输入源：`tank_vision/sources/`

| 文件/类 | 具体职责 |
| --- | --- |
| `base.py` / `FrameSource` | 迭代器抽象和上下文管理；`SourceError` 统一来源错误。 |
| `folder_source.py` / `FolderSource.__next__()` | 按稳定文件顺序读取图像，跳过/报告损坏输入，文件时间形成来源元数据。 |
| `video_source.py` / `VideoSource.__next__()` | OpenCV 解码视频；时间戳来自视频位置，释放 `VideoCapture`。 |
| `opencv_camera_source.py` / `OpenCVCameraSource` | 打开设备号或设备路径，设置期望尺寸，以主机时间构造帧。 |
| `vendor_camera_source.py` / `MindVisionCameraSource` | 动态载入 `mvsdk.py`/`libMVSDK.so`，选择序列号、分辨率、曝光和 Mono/BGR 格式，采图并安全释放 SDK buffer。 |
| `mindvision_probe.py` / `probe_mindvision_sdk()` | 只读搜索 SDK 根、头文件、库和架构，适合安装前诊断。 |

`MindVisionCameraSource._open()` 是非 ROS 直采的核心：枚举设备、严格选择设备、调用 SDK 配置、分配对齐缓冲并预热。`__next__()` 将 0.1 ms 设备 tick 换算为纳秒，同时附加曝光和增益。厂商 API 边界发生异常时转换为 `SourceError`。

相机参数优先检查：`sdk_root`、`serial_number`、`resolution_index`/`width`/`height`、`exposure_mode`、`exposure_us`、`warmup_frames`、`timeout_ms`。更换相机、USB 链路或分辨率后，不要沿用旧曝光和 ROI 参数。

### 3.4 传统焊缝检测：`tank_vision/seam/`

核心入口是 `classical_detector.py::ClassicalSeamDetector.detect(frame)`，实现顺序如下：

1. 可选 `Undistorter.apply()` 去畸变；开启但没有标定时返回丢失状态。
2. `make_image_view()` 裁剪 ROI，并按 `thumbnail_max_dimension` 缩放，同时保存到原图的坐标变换。
3. `_response()` 调用 `select_gray_channel()`、CLAHE、Gaussian/Bilateral 滤波，计算局部背景差和方向形态梯度，再按权重融合响应。
4. `_component_mask()` 使用 Otsu 或 adaptive threshold，闭/开运算，连通域过滤，并综合跨度、细长度、密度、居中性选择最佳组件。
5. `extract_component_centers()` 沿主方向逐行/列提取中心和宽度。
6. `fit_polynomial_ransac()` 对中心点做一次或二次多项式 RANSAC，并用内点重新拟合。
7. `calculate_confidence()` 综合覆盖率、内点率、残差、连续性、局部对比和位置合理性。
8. `seam_errors()` 在参考行计算横向偏差，并由多项式导数计算航向角。
9. `ImageView.to_original()` 把中心线和宽度恢复到原图像素坐标。

关键调参关系：

- 漏检：可适度降低 `minimum_response`、`response_percentile`、`min_component_span_ratio` 或 `min_confidence`；每次只改一组并用负样本复核。
- 伪检：提高响应/置信度门限、面积/跨度门限，或缩小 ROI。
- 断裂：增大 `morphology_kernel`/`morphology_iterations`，但过大会粘连相邻结构。
- 曲线拟合不稳：调整 `ransac_residual_px`、`max_fit_residual_px`、`fit_degree`；近直焊缝优先 degree 1。
- 输出偏差符号：竖直模式下 `lateral_error = seam_x - image_target_x`，正值表示焊缝位于目标右侧。

### 3.5 稳定焊缝跟踪器：`tank_vision/stable_seam_tracker.py`

这是面向 4096×3072 实时跟踪的独立算法，不等同于上一节的 `ClassicalSeamDetector`。

关键对象：

- `SeamWidthReference`（`seam_width_reference.py`）：载入带 SHA/分辨率信息的中心/宽度参考模型；`validate_image()` 防止错误分辨率/FOV混用；`at(y)` 给出参考中心和宽度。
- `BoundaryPair`：一条扫描带上的左边缘、右边缘、得分及各分项。
- `StableSeamTracker.process()`：完整单帧入口。
- `StableSeamResult` 与 `TrackingState`：输出 `LOST/ACQUIRING/TRACKING/DEGRADED`、左右边界、中心线、宽度、偏差、覆盖、拟合残差和失败原因。

`process()` 的真实实现流程：

1. `_prepare()`：验证输入与参考模型，按比例取得固定 ROI，转灰度，把 ROI 最大宽度缩到 `processing.max_roi_width`，Gaussian 平滑。
2. `_generate_candidates()`：对 x 方向做 Sobel；按 `scan_band_step_px` 生成水平扫描带；批量计算边缘响应和强度 profile；每带寻找正梯度左边缘和负梯度右边缘。
3. 配对：只接受宽度落在参考宽度 `width_min_scale..width_max_scale` 的组合，评分包含左右边缘强度、平衡、带内外对比、参考宽度、初始中心、时间中心和时间宽度。
4. guided 模式：`LOST/ACQUIRING` 用冻结参考模型，`TRACKING/DEGRADED` 用上一帧已接受模型，在预测边界附近局部搜索；可在局部无候选时回退全局搜索。
5. reference anchor：在参考边界附近注入窄窗口候选；可选 signed Sobel 或 normalized-step 响应。它是实验候选，配置文件明确不是生产默认。
6. `_select_path()`：动态规划连接各扫描带候选，累计单点得分并扣除中心、宽度、左右边界跳变和缺带成本。
7. `_fit()`：用 IRLS Huber 分别拟合中心二次曲线和宽度一次曲线，再取联合内点重拟合。
8. `_geometry()`：检查点数、覆盖率、有效跨度、最大连续缺带、突跳数、联合内点率、残差、路径置信度和拟合边界是否仍在 ROI。
9. `_temporal_accept()`：限制相邻有效帧横向、宽度和航向突变。
10. 时间状态机：连续成功达到 `acquiring_success_frames` 才进入 `TRACKING`；短暂失败在 `degraded_max_frames` 内输出衰减的预测；达到 `lost_frame_threshold` 清空历史模型并进入 `LOST`。
11. EMA：有效拟合以 `ema_alpha` 融合当前和历史系数，稳定输出。

最应调整的参数组：

- 性能：`max_roi_width`、`scan_band_step_px`、`top_k_edges_per_side`、`max_pairs_per_band`。宽度越小、步长越大、候选越少越快，但可能漏掉弱边界。
- 搜索范围：ROI 四个 ratio、`edge_percentile`、`edge_min_response`、`edge_min_separation_px`。
- 宽度先验：`reference.path`、`width_min_scale`、`width_max_scale`。参考文件必须来自同一镜头、工作距离和视场。
- 路径连续性：`center_transition_weight`、`width_transition_weight`、边界 transition、`skip_penalty` 和最大跳变 scale。
- 几何验收：`minimum_point_count/coverage/effective_span/inlier_ratio/path_confidence`、`maximum_fit_residual_px`。
- 时间稳定性：`ema_alpha` 越大响应越快但抖动更大；三个 jump 门限过严会把真实转弯判成失败，过松会跟随错误结构。
- guided：先验证无 guided 基线，再开启 `guided_search.enabled`；`reference_anchor_enabled` 应视为现场实验开关。

### 3.6 图像质量：`preprocessing/image_quality.py`

`assess_image_quality()` 计算平均亮度、过曝/欠曝比例、Laplacian 方差和对比度标准差，返回 `valid_image`、`valid_for_measurement`、`confidence_scale` 与 warnings。`RealtimeSeamPipeline` 会用 `confidence_scale` 下调焊缝置信度，低于 `seam.min_confidence` 时将结果改为未检测。

需要按真实曝光直方图调整 `underexposed_threshold`、`overexposed_threshold`、最大像素比例、`min_laplacian_variance` 和 `min_contrast_std`。这些阈值依赖光源、曝光、表面反光和分辨率，不应只凭合成图确定。

### 3.7 表面异常候选：`tank_vision/defects/`

关键函数：

- `TiledInference.tiles()`：按 `tile_size-overlap` 滑动，边缘 tile 用反射填充，记录有效宽高。
- `restore_detection()`：裁掉 padding 区，将 bbox、polygon 和 centroid 恢复到原图坐标。
- `ClassicalDefectBaseline._tile_mask()`：融合灰度局部背景差、Laplacian 纹理和 Lab 色差，做形态学并排除膨胀后的焊缝 mask。
- `_detect_tile()`：轮廓面积过滤、候选置信度、bbox/polygon/centroid 构造。
- `detect()`：遍历 tile、拼回全图 mask，最后 `non_maximum_suppression()` 去重。

该功能只能输出 `surface_anomaly`，不能宣称锈蚀、裂纹、凹坑或深度。重点参数是 `tile_size/overlap`、`anomaly_z_threshold`、`texture_z_offset`、`color_difference_threshold`、面积范围、`confidence_threshold`、`exclusion_dilation_kernel` 和 `nms_iou_threshold`。

`inspection_pipeline.py` 中 ONNX 路径当前只有安全的不可用占位 `_UnavailableModelDetector`：即使后端能加载，缺少明确模型输出解码适配器时也不会猜测检测结果。

### 3.8 推理后端：`tank_vision/inference/`

- `InferenceBackend` 定义 `available/load_model/infer/warmup/close` 和 tensor spec 契约。
- `OnnxBackend` 使用 ONNX Runtime，校验输入名、dtype、shape，只返回原始 tensor。
- `OpenCvDnnBackend` 使用 OpenCV DNN 加载 ONNX。
- `DRoboticsBpuBackend` 只允许 `target_arch=nash_m`，通过 adapter 对接官方 S100P runtime。
- `RknnBackend` 只通过 adapter 对接官方 RKNN runtime。

这些接口不包含任何通用 YOLO 解码猜测。真正接入模型必须同时提供：预处理顺序、颜色/归一化、输入布局、输出节点含义、后处理、量化校准集和目标架构元数据。

### 3.9 蓝色激光：`tank_vision/laser/`

`LaserLineDetector.detect()` 对 BGR 图计算 `B - 0.5*(G+R)`，与 HSV 蓝色门限取交集，形态学去噪后用 `column_weighted_centers()` 得到逐列灰度重心亚像素中心，再做一次/二次 RANSAC 并计算置信度。若提供 laser-off 帧，先由 `subtract_laser_frames()` 做配对帧相减。

`Profile3DInterface.reconstruct()` 当前是受保护接口：缺少相机内参、畸变、激光平面、相机到 base 外参或版本时返回 `uncalibrated`；即使字段齐全，当前源码仍返回 `triangulation_not_implemented_after_calibration_validation`。因此工程现在没有可用的激光三角测量三维输出。

### 3.10 三条流水线与写盘

`RealtimeSeamPipeline`（`realtime_pipeline.py`）：

- `_process_internal()` 依次补全相机元数据、评估质量、执行焊缝检测、合并 timing 和质量置信度。
- `submit()` 管理有界输入队列；默认容量 1、满时移除旧帧。
- `_worker()` 独立处理并输出；`rates` 给出输入/输出 Hz。

`InspectionPipeline`（`inspection_pipeline.py`）：

- `should_trigger()` 支持 `always`、`frame_interval`、`distance`、`external`。
- `submit()` 使用独立队列和 drop policy；默认每 15 帧触发且队满丢新任务。
- `_process_internal()` 隔离检测异常并记录 tile 变换。

`VisionPipeline`（`pipeline.py`）：同步兼容入口，先实时焊缝再精细异常，保证每次 `process(frame)` 都返回完整 `VisionResult`，适用于离线而非最高实时吞吐。

输出实现：

- `outputs/result_writer.py::ResultWriter.write()`：同步写 overlay、mask、JSON 和 CSV。
- `outputs/overlay.py::create_overlay()`：画中心线、候选框和状态。
- `outputs/async_writer.py::AsyncResultWriter`：后台写盘、队列策略、低磁盘保护、正常帧降频、异常前后窗口和 raw 编码。
- `snapshot_writer.py::write_snapshot()`：稳定跟踪人工快照。
- `failure_event_recorder.py::LiveSessionRecorder`：按状态变化/原因变化/最差指标等事件异步保存失败证据和逐帧指标。

低磁盘保护不会自动删除数据；`disk_low_space_threshold_mb` 触发后停止接收记录任务。生产中应同时监控 writer 的 dropped/failed 和剩余空间。

## 4. 视觉 ROS 2 功能包

### 4.1 `tank_vision_core`

文件：`ros2_ws/src/tank_vision_core/CMakeLists.txt`。

它没有算法副本，只用 `ament_python_install_package()` 把项目根唯一的 `tank_vision/` 安装进 ROS 环境，避免手工设置 `PYTHONPATH`。

### 4.2 `tank_vision_interfaces`

消息文件位于 `ros2_ws/src/tank_vision_interfaces/msg/`：

- `SeamState.msg`：兼容焊缝结果，中心线、横向/航向误差、宽度、原因和耗时。
- `SeamGeometry.msg`：稳定跟踪详细结果，左右边界、状态、预测标记、ROI、覆盖率、残差和丢帧计数。
- `Defect.msg`/`DefectArray.msg`：异常候选 bbox、polygon、面积、中心和后端。
- `VisionQuality.msg`：亮度、曝光比例、清晰度、有效性和警告。

所有消息的 `INTERFACE_VERSION=1`。结果 header 必须复制输入图像 header，才能保证关联。

### 4.3 `tank_vision_ros`

`vision_node.py::TankVisionNode`：

- 订阅 `image_topic` 和 `camera_info_topic`。
- `_image_callback()` 用 `CvBridge` 转 BGR，原样保留 ROS stamp/frame_id，立即同步执行 `RealtimeSeamPipeline.process()`。
- 立即发布 `SeamState` 和 `VisionQuality`；把缺陷任务提交到 `InspectionPipeline`。
- `_publish_inspection_results()` 由 10 ms timer 排空精检结果并发布 `DefectArray`。
- 只有配置允许且 debug publisher 有订阅者时才生成 overlay，减少无用开销。

`stable_seam_node.py::StableSeamNode`：

- 支持 `input_mode=ros_topic` 或 `mindvision_direct`；直采模式独占相机，不能同时运行另一个 MindVision 节点。
- `LatestFrameSlot` 始终保留最新帧，处理来不及时覆盖旧帧，并在消息中报告 dropped 计数。
- 发布 `/vision/front/seam_geometry`、兼容 `/vision/front/seam_state` 和可选 debug image。
- 提供 `/stable_seam_node/save_snapshot` Trigger 服务。
- 后台线程保存人工快照、自动失败事件、metrics 和 session summary。

`seam_debug_viewer.py::SeamDebugViewer` 用 OpenCV 窗口显示 debug image，并通过键盘触发参数/快照服务，仅用于调试。

主要 launch：

- `vision.launch.py`：通用传统视觉节点。
- `stable_seam_topic.launch.py`：订阅 ROS 图像的稳定跟踪。
- `stable_seam_direct.launch.py`：稳定跟踪器直接占用相机。
- `stable_seam_direct_guided_anchor.launch.py`：启用实验 guided-anchor 配置。

### 4.4 `inspection_rerun`

`rerun_bridge_node.py` 是低频、允许丢帧的观察桥，不控制硬件、不录 MCAP。主要适配器：

- `image_adapter.image_to_numpy()`：严格处理 encoding、step 和行 padding。
- `pointcloud_adapter.pointcloud_to_numpy()`：按字段读取 PointCloud2，stride/最大点数限流。
- `imu_adapter.imu_values()`、`tf_adapter.transform_values()`：转 Rerun 数据。
- `inspection_adapter`：将涡流/超声 reshape 为通道波形，将 FusionIndex 转标量。
- `timestamp_utils`：优先 header stamp，零时间用接收时刻并告警。
- `RerunSession`：统一 spawn/connect/save 三种输出模式和时间线。

默认图像 2 Hz、点云 1 Hz、波形 2 Hz；MCAP 仍保存完整原始频率。S100P GUI 不可用时应使用 `output_mode:=save` 生成 `.rrd`。

## 5. 视觉数据、训练与实验脚本

### 5.1 数据集与标定

- `dataset_tools.py::validate_records()` 检查 manifest 路径、标签、时间戳、尺寸和标定版本。
- `create_grouped_split()` 按 `(session_id,tank_id,scan_id)` 分组，防止同一扫描泄漏到 train/val/test。
- `calibration/camera_calibration.py` 读写相机矩阵、畸变和版本。
- `scripts/calibrate_camera.py` 用棋盘格标定；`square-size` 单位必须与后续外参和激光平面一致。
- `scripts/extract_dataset_frames.py` 从输入提帧；`validate_dataset.py`、`create_grouped_split.py` 是命令行封装。

### 5.2 训练接口

`training/` 只有 seam segmentation 和 defect segmentation 的配置约定、`export_onnx.py` 和 `validate_onnx.py`，没有训练器、数据、权重或真实性能结果。`export_onnx.py` 只调用用户明确提供的 exporter，`validate_onnx.py` 只验证 ONNX 可加载及 tensor 元数据。

### 5.3 稳定焊缝实验工具

脚本很多，但用途可按组理解：

| 类别 | 主要脚本 | 作用 |
| --- | --- | --- |
| 运行/回放 | `run_stable_seam_tracker.py`、`record_seam_validation_session.py`、`replay_seam_validation_session.py` | 文件、视频或相机运行，采集现场验证 session 并复现。 |
| 人工真值 | `annotate_weld_boundaries.py`、`validate_annotation_pair.py` | 标左右边界、保存 hash/原图坐标并校验。 |
| 参考模型 | `build_seam_width_reference.py`、`build_seam_geometry_reference.py` | 由人工边界建立随 y 变化的中心/宽度参考。 |
| 失败诊断 | `diagnose_*`、`audit_d10r2b_dataflow.py` | 拆解候选、配对分数、DP 前驱、reference selection、宽度尾部和光照问题。 |
| 消融实验 | `experiment_*`、`run_s1_p2d_simple_guided_search.py` | 比较方向、置信度、transition 权重、最小宽度、anchor 和 guided search。 |
| 验收 | `validate_stable_seam_tracker.py`、`evaluate_three_frame_ground_truth.py`、`validate_ros_seam_realtime.py`、`validate_guided_anchor_ros_candidate.py` | 离线真值、性能、ROS header/延迟与候选配置验证。 |

这些带 S1/D/P 编号的脚本多数是阶段实验，不是生产入口。生产配置升级应经过：固定数据集 → 基线报告 → 单参数消融 → 负样本回归 → 实时性能 → ROS 端到端 → 现场复测。

## 6. `physic_bev` 自研功能包

### 6.1 `mindvision_camera_driver`

核心文件：`mindvision_camera_driver/node.py`。

- `load_mvsdk()` 按参数、`MINDVISION_SDK_ROOT`、工作区和系统路径搜索 SDK，预载对应架构 `libMVSDK.so` 后导入 `mvsdk.py`。
- `_select_device()` 按字符串序列号选择；多个相机而未指定序列号会拒绝启动。
- `_select_resolution()` 打印 SDK 全部分辨率，只接受明确 index 或精确宽高匹配。
- `configure_exposure()`（`camera_settings.py`）严格区分 auto/manual，并先检查 SDK 范围。
- `_open_camera()` 根据传感器能力自动选择 Mono8 或 BGR8，设连续采集、曝光、预热并分配最大尺寸 buffer。
- `_acquire_processed()` 保证每次成功取得的 raw buffer 只释放一次，校验尺寸、媒体类型、step 和数据长度。
- `DeviceClockMapper.map()`（`clock_mapper.py`）展开 32 位 0.1 ms tick 的正常回绕，并锚定到 ROS 时域；小幅倒退不伪装为 wrap。
- `_capture_once()` 发布同 stamp/frame_id 的 `Image` 和未标定 `CameraInfo`。
- `close()` 按 pause → free buffer → uninit 顺序幂等清理。

私有话题是 `~/image_raw`、`~/camera_info`。节点若 `namespace=camera,name=front`，实际为 `/camera/front/image_raw`；`hardware_pipeline` 使用 `namespace=camera,name=camera`，实际为 `/camera/camera/image_raw`。它不会自动发布旧 D405 路径 `/camera/camera/depth/image_rect_raw`。

重点参数：SDK 根、序列号、精确分辨率、曝光/AE 范围、`warmup_frames`、`publish_rate_hz`、`timeout_ms`、QoS depth/reliability。4096×3072 BGR8 单帧约 37.7 MB，队列深度限制为 1～4；不要用深队列掩盖消费者性能问题。

时间戳是 `software_anchored_device_tick`，不是与 Livox/涡流硬同步。CameraInfo 的 K/D/P 当前为零，不能用于投影或测量。

### 6.2 `inspection_livox_adapter`

`livox_imu_adapter_node.cpp::LivoxImuAdapterNode` 订阅原始 IMU，将三轴线加速度乘 `acceleration_scale`（默认 9.80665），保留角速度、方向和协方差，检查零时间戳、非有限值和时间倒退，发布转换后的 `/livox/imu` 与 diagnostics。

只有确认原始驱动确实以 g 为单位时才能保持 9.80665；如果驱动版本已输出 m/s²，再乘一次会造成严重错误。

### 6.3 `eddy_driver`

`eddy_controller.cpp`：

- `open()` 配置 USB 串口；`startThread()/threadLoop()` 持续读字节。
- `threadLoop()` 是协议状态机，识别 `55 AA + 320-byte payload + 7E FE` ADC 帧和命令应答。
- `pushFrame()` 写入线程安全有界队列；`getNextFrame()` 供 ROS 节点逐帧排空。
- `sendCommand()` 生成 `AA + cmd + int32 little-endian + 55` 命令帧。

`eddy_driver_node.cpp`：

- `declareParameters()/loadParameters()` 处理设备、波特率、发布/查询频率和语义元数据。
- `publishData()` 排空解析帧，发布原始 payload、`EddyState` 和标准 `EddyCurrentFrame`。
- `publishCanonicalFrame()` 将 8×20 int16 ADC 转 channel-major float I；未知 Q 写 NaN。
- `querySettings()` 周期查询幅值/频率；`cmdCallback()` 接收 `[cmd,value]`。
- `publishDiagnostics()` 报告连接、接收、队列丢帧和未知语义。

STM32 帧没有硬件时间戳、Q、采样率、增益或提离量，因此 header 是主机解析时间，未确认字段保持 0/NaN。必须填写真实 `sensor_id/calibration_id` 后再做交付；`sampling_rate_hz/gain_db/lift_off_mm` 不得猜测。

### 6.4 `inspection_interfaces`

- `EddyCurrentFrame`：I/Q 波形及采样/增益/提离/质量元数据。
- `UltrasoundFrame`：channel-major int16 样本、声速、耦合度和接触力。
- `ProbeState`：接触力、提离、温度、耦合和状态。
- `RobotState`：线/角速度、电池、吸附电流、打滑和扫描状态。
- `FusionIndex`：锚点、各传感器时间、误差、有效性、插值位姿、资产坐标和质量标志。

波形数组长度应严格等于 `channel_count * sample_count`。

### 6.5 `inspection_sync`

`timestamped_ring_buffer.hpp::TimestampedRingBuffer<T>`：

- `insert()` 按时间戳排序插入，统计乱序/过晚/拒绝，并同时按最大年龄和最大数量裁剪。
- `nearest()` 找容差内最近消息。
- `range()` 取闭区间样本，用于 IMU 窗口。
- `bracket()` 取目标前后两条消息，用于位姿插值。

`inspection_sync_node.cpp::InspectionSyncNode`：

- 各订阅回调只入缓存；超声或涡流可作为 anchor。
- `consider_anchor()` 按 `anchor_decimation` 降采样；`enqueue_anchor()` 延迟 `fusion_process_delay_ms` 后交给 worker，给晚到消息留时间。
- `fuse_anchor()` 对相机/雷达做最近邻，对 IMU 取前后窗口，对 odometry 找 bracket。
- `interpolate_pose()`（`sync_math.hpp`）对位置线性插值、姿态 quaternion SLERP，并归一化。
- `quality_flags()` 用 bit 0/1/2/3 表示 camera/lidar/imu/pose 缺失；`invalid_reason()` 给文字原因。
- `update_clock_match()` 支持固定软件 offset 和实验性窗口均值自适应更新。
- `publish_diagnostics()` 每秒输出各缓存长度、年龄、乱序、late、pending、offset 和成功/失败计数。

关键参数调整顺序：

1. 先保持 `software_clock_match_enable=false`，采集真实误差分布。
2. `fusion_process_delay_ms` 应大于常见传输抖动，但过大会增加索引延迟。
3. `camera_tolerance_ms/lidar_tolerance_ms` 过小导致 invalid，过大会错误关联相邻帧。
4. IMU 窗口必须在锚点两侧均有样本才有效。
5. `pose_max_gap_ms` 应结合 odometry 频率设置。
6. 自适应 offset 只有在确认各 stamp 同一时钟模型、误差稳定且有独立验证后才能启用。

### 6.6 `inspection_mock_sensors`

`mock_sensors_node.cpp::MockSensorsNode` 用固定随机种子和独立 timer 发布兼容图像/CameraInfo、PointCloud2、IMU、Odometry、RobotState、UltrasoundFrame、EddyCurrentFrame、ProbeState 和 TF。它用于 CI、录包、同步和工具测试，不代表真实硬件精度、吞吐或时间同步。

参数集中在 `inspection_bringup/config/mock_sensors.yaml`，包括各频率、图像尺寸、点数、波形通道/样本数和机器人速度。

### 6.7 `inspection_tools`

console scripts 由 `setup.py` 注册：

- `validate_bag` → `validate_bag.py::validate()`：使用 `rosbag2_py` 顺序读包，检查必需话题、消息数量、stamp 单调性、Image/PointCloud2 字段、波形数组和 FusionIndex 误差，并生成报告。
- `finalize_bag` → `finalize_bag.py::finalize()`：确认包已关闭，生成 manifest 和 SHA-256，可复制到交付目录。不要对仍在录制的目录执行。
- `estimate_storage`：按分辨率、字节/像素、频率、时长和各传感器速率估算容量。
- `inspect_latest`：寻找最新“已关闭”的 bag，读取 metadata 并打印话题/频率概况。
- `depth_to_pointcloud`：由真正的 depth image 和有效 CameraInfo 内参投影 XYZ；当前 MindVision mono8 不是深度，禁止接入该工具。

### 6.8 `inspection_bringup`

主要 launch：

- `hardware_pipeline.launch.py`：MindVision、MID-360、IMU 单位适配、可选涡流、可选 sync、静态 TF 和 MCAP 总入口。
- `record_only.launch.py`：仅录制现有话题。
- `mock_pipeline.launch.py`：模拟传感器 + sync + 可选录制。
- `baseline_capture/replay.launch.py`：定时基线采集/回放。
- `eddy_sync_capture/replay.launch.py`：涡流锚点链路采集/回放。
- `rough_calibration_view.launch.py`：RViz 查看粗外参。

录制配置：`record_topics.yaml` 定义 14 个目标话题；debug 分片 2 GiB/30 s，production 分片 10 GiB/900 s，cache 1 GiB；`mcap_writer_options.yaml` 使用 Zstd Fast 和 64 MiB chunk。

`rough_extrinsics.yaml` 的 MID-360→camera 变换明确标记 `production_valid:false`，只能粗可视化，不能做缺陷物理定位。

## 7. 上游定位/驱动功能包

### 7.1 `FAST-LIVO2`（ROS 包名 `fast_livo`）

构建目标与文件对应：

- `preprocess.cpp` / `Preprocess`：不同雷达点格式预处理、时间/特征信息整理。
- `IMU_Processing.cpp` / `ImuProcess`：IMU 初始化、协方差/偏置、状态传播、点云去畸变。
- `voxel_map.cpp`：体素地图、平面匹配、邻域和增量更新。
- `frame.cpp`、`visual_point.cpp`、`vio.cpp`：图像金字塔、视觉地图点、patch 直接法残差和视觉更新。
- `LIVMapper.cpp` / `LIVMapper`：ROS 参数、订阅缓存、雷达/IMU/图像同步、LIO/VIO 调度、里程计/点云/图像/路径发布和保存。
- `main.cpp`：初始化 ROS，构造 mapper 并 spin。

`config/livo.yaml` 关键参数：输入话题、LiDAR→camera 外参 `Rcl/Pcl`、相机/IMU时间偏移、点云滤波、VIO patch/金字塔/网格、IMU 噪声、LIO 体素地图、局部地图、发布与 PCD/图像保存。

高风险参数是外参 `Rcl/Pcl`、`img_time_offset`、IMU 噪声、`filter_size_surf`、`voxel_size` 和各 topic。它们错误时算法可能仍运行但轨迹错误。

当前配置存在必须人工确认的矛盾：实际 `common.img_en: 1`，但 `img_time_offset` 上方注释称“当前 img_en=0、运行 LIO 模式”。运行前应以实际目标决定：

- 只做 LIO：将 `img_en` 明确设为 0，并确认节点不会等待图像。
- 做 LIVO：保持 1，确保 `/camera/camera/color/image_raw` 真有发布者，并重新标定 D405/MindVision 对应外参与时间偏移。

### 7.2 `top_pkg`

这是旧的 MID-360 + RealSense D405 一键入口：

- `bringup_sensor.launch.py` 读取 `config/bringup.yaml`，按开关包含 Livox 官方 launch 和 RealSense `rs_launch.py`。
- `bringup.launch.py` 再包含 FAST-LIVO2 mapping，并统一启动 RViz。

它仍启动 RealSense，而不是新的 MindVision 采集链。要做当前生产采集，优先使用 `inspection_bringup/hardware_pipeline.launch.py`；要跑原 FAST-LIVO2 D405 建图，才使用 `top_pkg`。

### 7.3 其他上游包

- `livox_ros_driver2`：连接 MID-360，发布 `/livox/lidar` 和原始 IMU；配置 JSON 中 IP 必须与设备/主机网段一致。
- `realsense-ros`：D405 上游驱动，仅属于旧 FAST-LIVO2 路径。
- `rpg_vikit`：FAST-LIVO2 视觉几何依赖。
- `Sophus`、`Livox-SDK2`：第三方数学与硬件 SDK，不应在业务调参时修改。

## 8. 当前源码中必须先处理或明确的事项

1. **相机话题不一致**：新 MindVision 驱动发布 `/camera/camera/image_raw`（hardware launch 的 namespace/name），而 `hardware_pipeline` 默认 `camera_image_topic`、sync、录制配置和部分文档仍指向 `/camera/camera/depth/image_rect_raw`。必须通过 launch remap 或统一配置修正，否则 sync/MCAP 会缺图像。
2. **相机型号/编码描述不一致**：架构文档写 GM Mono8，当前安装配置文件名和 README 又包含 GC BGR8，hardware launch 默认序列号也不同。以 `CameraGetCapability().bMonoSensor` 和实际序列号为准。
3. **曝光参数链不完整且默认值会冲突**：`hardware_pipeline` 只传 `exposure_us`，未传 `exposure_mode` 和 AE 上下限；驱动默认 `exposure_mode=auto`，但 launch 默认又传 `exposure_us=30000`。`configure_exposure()` 明确要求 auto 模式的 `exposure_us` 必须为 0，因此按当前源码直接使用默认值会导致相机启动失败。临时自动曝光运行要传 `mindvision_exposure_us:=0.0`；若要求固定曝光，应修改 launch，显式传 `exposure_mode:=manual` 和合法曝光值。
4. **CameraInfo 未标定**：K/D/P 为零。禁止称图像已校正、禁止相机投影、毫米换算和相机-雷达精确融合。
5. **粗外参不是生产外参**：`rough_extrinsics.yaml` 只有卷尺/安装方向结果。
6. **时钟不是硬同步**：MindVision 是软件锚定设备 tick，涡流是主机解析时间；software offset 只能补常量，不能替代硬件触发/PTP。
7. **FAST-LIVO2 图像开关注释冲突**：运行前明确 LIO 还是 LIVO。
8. **最终 BEV 未实现**：Rerun 中的 BEV topic 是预留显示接口，不是当前算法输出。
9. **模型推理未闭环**：没有权重、模型解码器、板端运行时实测或真实精度，不能把 backend 接口当作已完成检测模型。
10. **绝对路径**：部分稳定跟踪和文档含 `/home/sunrise/...` 或历史 `/home/dts/...`；迁移机器时改 launch/YAML 参数，不要复制旧路径。
11. **生成目录不是源码**：修改 `build/` 或 `install/` 会在下次构建消失，所有改动必须落到 `src/` 或项目根 Python 包。

## 9. 从零实现/运行的推荐指令

下面命令均以当前路径 `/home/sunrise/east_eleair` 为基准。硬件命令必须在有设备权限和正确 ROS 环境的主机执行。

### 9.1 视觉离线最小闭环

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection
python3 -m pip install -e .
python3 scripts/generate_synthetic_samples.py
python3 -m tank_vision.cli \
  --config configs/default.yaml \
  --input sample_data/synthetic \
  --output output_documented_run
```

输出应包含 `overlays/`、`masks/`、`json/`、`summary.csv` 和 `performance_summary.json`。

### 9.2 视觉三通路实时软件演示

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection
python3 scripts/run_three_path_demo.py \
  --config configs/default.yaml \
  --input sample_data/synthetic \
  --output output_three_path_documented \
  --pace-hz 15
```

### 9.3 稳定焊缝离线验证

先查看实际 CLI 参数，避免阶段脚本参数随实验变化：

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection
python3 scripts/run_stable_seam_tracker.py --help
python3 scripts/validate_stable_seam_tracker.py --help
```

典型运行：

```bash
python3 scripts/run_stable_seam_tracker.py \
  --config configs/s100p_stable_seam_4096x3072.yaml \
  /absolute/path/to/images \
  --output output/stable_seam_check
```

参考模型或输入分辨率不匹配时应停止，不要用 `same_fov_scaled=true` 强行绕过，除非确认只是同一视场等比例缩放。

### 9.4 视觉 ROS 2 构建与运行

Jazzy/x86：

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch tank_vision_ros vision.launch.py
```

稳定跟踪 ROS topic 模式：

```bash
ros2 launch tank_vision_ros stable_seam_topic.launch.py \
  image_topic:=/camera/front/image_raw \
  publish_debug_image:=true
```

保存一次快照：

```bash
ros2 service call /stable_seam_node/save_snapshot std_srvs/srv/Trigger '{}'
```

### 9.5 测试

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection
python3 -m compileall -q tank_vision scripts training ros2_ws/src
python3 -m pytest -m "not camera and not ros2 and not s100p and not rk3588" -q
```

全量测试会因没有相机、板卡、runtime 或 DDS 环境而合理 skip/fail，执行前确认 marker 条件。

### 9.6 `physic_bev` 构建

生产采集文档指向 Ubuntu 22.04/ROS 2 Humble；旧 FAST-LIVO2 README 又记录过 Jazzy。不要在未确认发行版时混用已有 `install/`：

```bash
cd /home/sunrise/east_eleair/physic_bev
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只构建自研采集包：

```bash
colcon build --symlink-install --packages-up-to \
  inspection_interfaces mindvision_camera_driver inspection_livox_adapter \
  eddy_driver inspection_sync inspection_mock_sensors inspection_tools inspection_bringup
```

### 9.7 无硬件验证采集、同步和 MCAP

```bash
cd /home/sunrise/east_eleair/physic_bev
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch inspection_bringup mock_pipeline.launch.py record:=true
```

另一个终端检查：

```bash
ros2 topic echo /derived/inspection/fusion_index --once
ros2 topic echo /diagnostics --once
```

### 9.8 MindVision 独立启动

```bash
cd /home/sunrise/east_eleair/physic_bev
source /opt/ros/humble/setup.bash
source install/setup.bash
export MINDVISION_SDK_ROOT=/home/sunrise/east_eleair/MindVisionlinuxSDK
ros2 launch mindvision_camera_driver mindvision_camera.launch.py
```

验证真实话题名和编码：

```bash
ros2 topic list | grep camera
ros2 topic echo /camera/front/camera_info --once
ros2 topic hz /camera/front/image_raw
```

### 9.9 真实硬件联合采集

在修正/显式设置相机话题后运行。为了避免当前默认不一致，下面明确把 sync 输入设为新驱动实际话题；录制 topic YAML 也应同步更新或额外录制该话题：

```bash
cd /home/sunrise/east_eleair/physic_bev
source /opt/ros/humble/setup.bash
source install/setup.bash
export MINDVISION_SDK_ROOT=/home/sunrise/east_eleair/MindVisionlinuxSDK

ros2 launch inspection_bringup hardware_pipeline.launch.py \
  start_mindvision:=true \
  camera_image_topic:=/camera/camera/image_raw \
  mindvision_exposure_us:=0.0 \
  mindvision_publish_rate_hz:=10.0 \
  start_mid360:=true \
  mid360_config_path:=/absolute/path/to/MID360_config.json \
  start_eddy:=false \
  start_sync:=false \
  publish_rough_extrinsics:=false \
  record:=true \
  debug_mode:=true \
  output_directory:=/absolute/path/to/bags/run_001
```

开启涡流和同步前，必须填真实设备元数据：

```bash
ros2 launch inspection_bringup hardware_pipeline.launch.py \
  start_mindvision:=true \
  camera_image_topic:=/camera/camera/image_raw \
  mindvision_exposure_us:=0.0 \
  start_mid360:=true \
  start_eddy:=true \
  eddy_device:=/dev/ttyACM0 \
  eddy_sensor_id:=ACTUAL_SENSOR_ID \
  eddy_calibration_id:=ACTUAL_CALIBRATION_ID \
  start_sync:=true \
  anchor_topic:=/inspection/eddy_current/raw \
  software_clock_match_enable:=false \
  asset_id:=ACTUAL_ASSET \
  course_id:=ACTUAL_COURSE \
  plate_id:=ACTUAL_PLATE \
  weld_id:=ACTUAL_WELD \
  record:=true \
  output_directory:=/absolute/path/to/bags/run_eddy_001
```

### 9.10 MCAP 校验与固化

```bash
source /home/sunrise/east_eleair/physic_bev/install/setup.bash
ros2 bag info /absolute/path/to/bag
ros2 run inspection_tools validate_bag /absolute/path/to/bag --help
ros2 run inspection_tools finalize_bag /absolute/path/to/bag --help
```

先用 `--help` 核对必填 ID/输出参数；确认 rosbag 进程退出且 metadata 完整后才能 finalize。固化后不要再修改 MCAP 或 manifest，否则 SHA-256 失效。

### 9.11 FAST-LIVO2 旧 D405 路径

```bash
cd /home/sunrise/east_eleair/physic_bev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch top_pkg bringup_sensor.launch.py enable_rviz:=false
ros2 launch top_pkg bringup.launch.py enable_rviz:=true
```

实际部署若是 Humble，需使用 Humble 构建产物并确认所有依赖版本。运行前检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /camera/camera/color/image_raw
ros2 param get /fastlivo_mapping common.img_en
```

### 9.12 Rerun 可视化

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection
source /opt/tros/humble/setup.bash
source .venv_rerun/bin/activate
source ros2_ws/install/setup.bash
ros2 launch inspection_rerun rerun_live.launch.py output_mode:=save \
  save_path:=/home/sunrise/east_eleair/tank_vision_inspection/output/rerun/live.rrd
```

回放：

```bash
ros2 launch inspection_rerun rerun_replay.launch.py \
  bag_path:=/absolute/path/to/bag \
  output_mode:=save \
  save_path:=/absolute/path/to/replay.rrd \
  playback_rate:=1.0 \
  loop:=false
```

## 10. 上线前验收清单

- 明确实际相机型号、序列号、Mono/BGR、USB 速率、分辨率、曝光和帧率。
- 统一 MindVision 话题名，确认 image、CameraInfo、sync 和 record 四处一致。
- 完成相机内参，并在同一镜头/焦距/工作距离下重新建立焊缝参考。
- 若做融合，完成相机—LiDAR—base 精确外参和独立时间同步验证。
- 对正样本、负样本、弱光、反光、遮挡、转弯分别建立不可变验证集。
- 记录 P50/P95/max 延迟、输入/输出 Hz、drop、writer failed、磁盘速率和剩余容量。
- 检查 `FusionIndex.quality_flags` 分布，而不只看是否有消息。
- 涡流/超声所有 ID、采样率、增益、时钟语义和标定版本必须来自真实设备配置。
- FAST-LIVO2 明确 LIO/LIVO 模式，验证 topic、外参、时间偏移和 IMU 单位。
- MCAP 停止后执行 validate，再 finalize/hash；Rerun 仅作观察，不作数据真源。
- 所有结论区分“合成测试、模拟节点、实验候选、真实硬件验证、生产标定”五种证据等级。

## 11. 建议的实施顺序

1. 先修正相机话题、型号/编码和曝光参数传递，使 MindVision 单机 1000 帧稳定。
2. 完成相机内参和固定光学条件，重建稳定焊缝 reference。
3. 用人工边界数据调稳定跟踪，冻结生产候选配置。
4. 单独验证 Livox 点云/IMU 单位和退出行为。
5. 单独验证涡流协议、采样语义和设备 ID。
6. 联合录 MCAP，但先不开自适应时钟；离线分析时间差。
7. 调整 sync 容差/固定 offset，再验证 FusionIndex 有效率。
8. 完成精确外参后再启用物理融合、投影或 BEV。
9. 最后接入明确模型和板端 runtime；先保证输出解码与 x86 黄金结果一致，再测 S100P/RK3588 性能。

按这个顺序可以保证每一步都有可验证输入/输出，也不会把尚未标定或尚未实现的接口误当成物理测量能力。
