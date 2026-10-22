# 测试验证说明

验证日期：2026-08-20（Asia/Shanghai）。工程路径：`/home/dts/visoin_track/tank_vision_inspection`。

## 1. 修改前基线

- `compileall`：通过。
- `pytest`：28 passed，1.49 s，与上一阶段的28项基线一致。
- 合成数据：生成8张；离线demo处理8/8、焊缝6/8、异常候选2个。
- 本轮修改前隔离实测平均算法耗时：88.81 ms/帧；上一阶段记录为108.09 ms/帧，差异19.28 ms，判定为运行负载/环境波动，不回写或伪造历史值。
- 原输出：8 overlay、8 mask、8 JSON、8行CSV；另存到 `output_baseline_20260820/`，没有覆盖旧 `output/` 唯一副本。
- Git：项目和上级目录起初都不是有效仓库；上级 `/home/dts/visoin_track/.git` 是空的40-byte只读占位目录，没有对象。未删除或修改它。在项目目录初始化独立仓库，原始MVP基线提交为 `e8173e2`，没有远程。
- 环境：x86_64、Linux 6.14、Python 3.13.9、OpenCV 4.13.0、NumPy 2.3.5、PyYAML 6.0.3、ROS 2 Jazzy。系统 `/usr/bin/python3` 为ROS使用的Python 3.12环境，并有OpenCV 4.6/NumPy 1.26.4。
- 未找到：MindVision SDK、`CameraApi.h`、`libMVSDK.so`、ONNX Runtime、`hbm_runtime`、`hobot_dnn`、RKNN Runtime、Python 3.10和ROS 2 Humble。

## 2. 新增和修改文件

- 三通路与性能：`tank_vision/realtime_pipeline.py`、`inspection_pipeline.py`、`metrics.py`，并把 `pipeline.py` 保留为确定性兼容入口；检测器增加分阶段计时和tile变换记录。
- 异步记录：`tank_vision/outputs/async_writer.py`，同步writer/JSON/CSV增加阶段字段、flush/close、写盘隔离、低磁盘保护，以及正常/异常/前后窗口原图策略。
- 平台与推理：`tank_vision/platforms/`、`tank_vision/inference/`、`configs/platforms/{x86,s100p,rk3588}.yaml`；原ONNX导入路径保持兼容。
- 相机：MindVision只读SDK探测、`MindVisionCameraSource`安全接入位置、相机配置模板、probe和smoke-test脚本；没有猜测任何厂商函数名。
- 工程：`requirements/`分层、Python版本上限、pytest markers、`tox.ini`、Humble Docker验证文件和扩展后的 `.gitignore`。
- 数据/训练：`dataset/` Schema/标签/空目录、`dataset_tools.py`、校验/分组切分脚本；`training/` 只有文档、配置、显式exporter和ONNX元数据检查，无数据或权重。
- ROS：新增 `tank_vision_core` ament安装适配包；ROS节点实时发布seam/quality并异步精检；消息接口常量 `INTERFACE_VERSION=1`；纯核心不导入rclpy。
- 文档：更新 `README.md`，新增 `ROADMAP.md` 和本文件。
- 测试：新增性能、队列、三通路、writer失败、平台/后端、相机跳过、数据集、ROS元数据和可选ROS端到端测试。

## 3. 架构调整

实时循迹 `RealtimeSeamPipeline` 默认容量1和 `drop_oldest`，只做quality、ROI/缩略图、焊缝检测与像素几何；它不分块检测、不生成overlay、不写盘。同步 `process()` 供确定性调用，异步 `submit()` 供实时latest-frame语义；统计输入/输出频率、队列深度和丢帧。

精细检测 `InspectionPipeline` 使用独立有界队列和线程，支持always、帧间隔、编码器距离、外部事件触发。1024/128分块、边缘补齐、NMS和原图坐标恢复保留，`TileTransform` 记录每块的原点和有效尺寸。默认每15帧触发一次，因此不要求每个相机帧都精检。

记录 `AsyncResultWriter` 使用第三条有界队列。overlay、mask、JSON、CSV、原图压缩和磁盘操作都在后台线程；写盘失败被记录为warning/failed，不传播到实时算法。正常帧降频、异常原图、压缩前后窗口和低磁盘停止记录均由 `storage` 配置控制，不自动删除文件。

离线CLI仍使用 `VisionPipeline + ResultWriter` 同步顺序处理，8张有效输入得到8份稳定有序输出，不采用实时丢帧策略。

## 4. 平台兼容

- x86：OpenCV CPU、核心/三通路、异步writer、Jazzy构建和ROS本机端到端已真实执行。ONNX接口已实现，但本机无ONNX Runtime和模型，因此没有真实ONNX推理。
- S100P：`DRoboticsBpuBackend` 优先探测 `hbm_runtime`，兼容探测 `hobot_dnn`；当前返回安全不可用和具体原因。Nash-m/input-format/目标板元数据与fake adapter接口测试通过；这不是BPU硬件推理。未运行S100P/Humble。
- RK3588：统一 `RknnBackend`、可选导入和错误目标校验已完成；Runtime缺失不影响其他模块。未运行RK3588或RKNN模型。
- MindVision：probe实际返回SDK缺失，smoke test安全退出；没有相机帧、丢帧率或硬件时间戳测试结果。
- Python：3.13.9和系统3.12的56项非硬件测试都真实通过。3.10/3.11未安装，只有tox/Docker验证方式，不能声明通过。
- ROS：Jazzy三个包真实构建。消息只使用Humble已有接口，但Humble容器/板端尚未执行。

## 5. 性能结果

### 确定性离线8帧（含实时算法和每帧精检，不含写盘）

- 修改前本轮实测平均：88.81 ms；上一阶段历史值：108.09 ms。
- 修改后平均：88.03 ms；P50 72.12 ms；P90 120.17 ms；P95 169.56 ms；最大218.94 ms；平均算法FPS 11.36。
- 平均写盘：1.86 ms，未计入算法时间。
- 最耗时三个阶段：`defect_inference_ms` 51.74 ms、`seam_geometry_ms` 13.93 ms、`seam_detect_ms` 11.42 ms。
- 结果仍为8/8、焊缝6/8、异常2个；原JSON顶层字段缺失数为0，新增加 `metrics`。

### 三通路80帧（8张合成图确定性重复10次，15 Hz输入，周期精检并发）

- 实时通路：平均31.31 ms；P50 32.23 ms；P90 45.08 ms；P95 50.37 ms；最大87.83 ms。
- 实时算法FPS：31.94；输入15.01 Hz；输出15.25 Hz；处理80、丢弃0、失败0、最大队列深度1。
- 实时端到端：平均31.58 ms；P95 50.52 ms；最大88.08 ms。
- 实时最耗时三个阶段：`seam_geometry_ms` 13.73 ms、`seam_detect_ms` 11.31 ms、`quality_ms` 3.77 ms。
- 精检：触发6、按策略跳过74、丢弃0；平均94.45 ms，P95 202.53 ms。它不计入实时循迹延迟。
- writer：处理80、丢弃0、失败0；平均磁盘写入2.06 ms。有限文件demo在算法结束后分批组装同帧结果，因此writer queue等待时间不代表相机实时流吞吐；它同样没有计入实时算法时间。

结论：这组x86合成数据软件基准达到平均≤50 ms、P95≤66.7 ms和≥15 Hz三个目标，但最大单帧87.83 ms仍需保留并在真实相机/真实数据上继续观察。结果不外推到4096×3072真实图、S100P、RK3588或工业相机。

## 6. 测试结果

- `compileall`：通过，覆盖core/scripts/tests/training/ROS Python源码。
- 默认非硬件pytest：56 passed，2 deselected，Python 3.13.9。
- 当前环境完整pytest：56 passed，2 skipped；skip分别为MindVision SDK/相机缺失和默认未开启DDS端到端。
- 系统Python 3.12非硬件pytest：56 passed，2 deselected。
- 离线demo：8/8；8 overlays、8 masks、8 JSON、8 CSV rows。
- 三通路80帧：80 realtime结果、80 writer结果，writer正常flush/close。
- ROS构建：`tank_vision_core`、`tank_vision_interfaces`、`tank_vision_ros`，3 packages finished。
- ROS端到端：显式开启后1 passed；确实发布图像并断言seam/quality的stamp和frame_id。沙箱禁止UDP socket枚举并打印FastDDS警告，但本地进程内通信和断言完成；未验证跨机DDS网络。
- 数据集：Schema/路径/标签/时间戳/尺寸/标定版本和group leakage单测通过；空示例manifest工具运行通过（0样本不代表有真实数据）。
- Git：最终执行 `git diff --check`、`git fsck`；无远程和无模型/SDK二进制提交。

## 7. 未完成内容

- 未安装或审查MindVision官方SDK，未接相机，未做1000帧稳定性、曝光/ROI/触发/硬件时间戳验证。
- 无S100P/RK3588板卡及官方Runtime，未做BPU/RKNN模型加载、速度、功耗或精度测试。
- 无真实数据/标注，没有训练、权重、真实Precision/Recall/F1/IoU或验收阈值。
- 无ONNX Runtime和具体模型，未验证导出、解码器、算子兼容或量化精度。
- 未运行Ubuntu 22.04/Python 3.10/ROS 2 Humble；只提供tox与Docker方案。
- 无完整相机—激光—base标定，不输出毫米焊缝宽度、余高、凹坑深度或点云，也没有真实三维精度。
- 未接MID-360、IMU、LIO、编码器和涡流；未验证跨机DDS网络。

## 8. 运行命令

本机单元/全量测试：

```bash
cd /home/dts/visoin_track/tank_vision_inspection
python3 -m compileall -q tank_vision scripts tests training ros2_ws/src/tank_vision_ros
python3 -m pytest -m "not camera and not ros2 and not s100p and not rk3588" -q
python3 -m pytest -q
```

离线demo：

```bash
python3 scripts/generate_synthetic_samples.py
python3 -m tank_vision.cli --config configs/default.yaml \
  --input sample_data/synthetic --output output
```

实时三通路与80帧性能复现：

```bash
python3 scripts/run_three_path_demo.py --config configs/default.yaml \
  --input sample_data/synthetic --output output_three_path --pace-hz 15
python3 scripts/run_three_path_demo.py --config configs/default.yaml \
  --input sample_data/synthetic --output output_profile --pace-hz 15 --repeat 10
```

平台与MindVision探测：

```bash
python3 scripts/detect_platform.py
python3 scripts/probe_mindvision_sdk.py
python3 scripts/test_mindvision_camera.py --config configs/camera_mv_suf1205gc_t.yaml
```

数据集校验和分组切分：

```bash
python3 scripts/validate_dataset.py dataset/manifest.jsonl --dataset-root dataset
python3 scripts/create_grouped_split.py dataset/manifest.jsonl dataset/splits/grouped.json
```

ROS节点和端到端：

```bash
cd /home/dts/visoin_track/tank_vision_inspection/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch tank_vision_ros vision.launch.py
```

```bash
cd /home/dts/visoin_track/tank_vision_inspection
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOG_DIR=/tmp/tank_vision_ros_logs
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
TANK_VISION_RUN_ROS2_E2E=1 /usr/bin/python3 -m pytest tests/test_ros2_integration.py -q -s
```

MCAP录制：

```bash
ros2 bag record -s mcap /vision/image_raw /camera/camera_info \
  /vision/debug_image /vision/seam_state /vision/defects /vision/quality
```

## 9. 下一步唯一优先任务

安装并审查迈德威视x86官方SDK。原因：当前三通路、记录、平台和数据规范已经可运行，但没有受控的真实采集入口；先用官方x86示例确认SDK、USB3、像素格式、曝光/ROI/触发和硬件时间戳，才能安全进入固定光学、1000帧稳定性和真实数据采集。此步骤也能尽早锁定后续S100P需要的ARM64 SDK版本，避免在假相机接口上训练或调参。
