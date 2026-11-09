# S0-C ROS 视觉端到端集成验证

**验证日期：** 2026-09-03
**工程目录：** `/home/sunrise/east_eleair/tank_vision_inspection`
**关联 HEAD：** `33e59417a17e764a127c7d91be1b5098f9a348ba`
**验证性质：** synthetic ROS integration；无硬件、无真实采集、无生产精度验收。

## 1. 结论

本轮扩展了 `tests/test_ros2_integration.py`，使用真实 `TankVisionNode`、真实 ROS 发布/订阅节点和真实 classical 视觉算法处理 pytest 临时目录中的合成图像。

三次有效独立运行均通过：

```text
3 passed in 1.47s
```

覆盖：

1. 正常焊缝图：焊缝检测、质量消息、空缺陷数组和 debug image；
2. 表面异常图：classical `surface_anomaly` 输出、置信度、面积、边界框和坐标；
3. 三帧受控关联：不同图像、时间戳和 `frame_id` 与四类输出逐帧对应，异常结果没有绑定到相邻正常帧。

本轮不代表真实相机采集、MCAP 录制、硬件同步、BPU 推理、标定、真实 BEV、长时间稳定性或机器人控制验收。

## 2. 修改范围

本轮修改：

```text
tests/test_ros2_integration.py
docs/S0_C_ROS_VISUAL_INTEGRATION.md
```

未修改视觉算法、生产配置、ROS 消息定义、节点队列策略或任务执行器。

临时配置由测试从 `configs/default.yaml` 读取后写入 pytest `tmp_path`，只覆盖：

```yaml
inspection:
  enabled: true
  trigger_mode: always
defect:
  backend: classical
```

其他算法阈值保持默认配置值；`configs/default.yaml` 本身未修改。

## 3. 测试实现要点

### 3.1 显式启用和环境门控

保留 `TANK_VISION_RUN_ROS2_E2E=1` 的显式启用要求。

- 未设置该变量时测试跳过；
- `ROS_DISTRO` 不是 `humble` 或 `AMENT_PREFIX_PATH` 缺失时测试跳过；
- 启用后，如果节点初始化、DDS 发现、算法处理或断言失败，则测试失败，不伪装成跳过；
- 缺少 `rclpy`、`tank_vision_ros` 或 `cv_bridge` 时按缺少 ROS 环境处理。

### 3.2 真实节点和真实算法

每个测试都实例化真实 `TankVisionNode`，并创建独立的测试发布/订阅节点：

- 输入：`/vision/image_raw`；
- 输出：`/vision/seam_state`；
- 输出：`/vision/quality`；
- 输出：`/vision/defects`；
- 输出：`/vision/debug_image`。

测试通过 ROS 参数 `config_file` 将临时配置传给节点，没有替换 `RealtimeSeamPipeline`、`InspectionPipeline` 或 defect detector。

启动前通过 DDS endpoint 计数等待输入发布器和四个输出发布器匹配，并设置发现超时；没有用固定 sleep 或重复发送同一帧掩盖发现问题。

### 3.3 后端和内部失败检查

测试同时检查：

```text
vision.config.inspection.enabled == True
vision.config.inspection.trigger_mode == "always"
vision.config.defect.backend == "classical"
defect_detector.backend_name == "classical_surface_anomaly"
```

对于空缺陷数组，测试不读取或提前消费节点内部结果队列，而是在收到外部 `DefectArray` 后读取公开统计接口：

```text
vision.inspection_pipeline.stats.summary()
```

并检查：

- `processed_frames` 至少达到预期数量；
- `failed_frames == 0`；
- classical detector 的 `last_warning is None`。

当前 `DefectArray` 不包含检测警告字段，因此外部订阅者不能仅凭空数组判断内部检测是否成功；测试使用节点内部可访问统计完成这项边界检查。

## 4. 场景结果

### 4.1 正常焊缝图

样本由现有 `tank_vision.synthetic.seam_image()` 生成，尺寸为 640×480，编码为 BGR8。

通过条件及结果：

- 收到四类输出各 1 条；
- 四条消息 Header 均与输入的 `sec`、`nanosec`、`frame_id` 完全一致；
- `SeamState.detected == true`；
- `VisionQuality.valid_image == true`；
- `VisionQuality.valid_for_measurement == true`；
- 质量数值均为有限值；
- `DefectArray.defects` 为空；
- 内部 `failed_frames == 0`、classical detector 无 warning；
- debug image 为 `bgr8`，尺寸 640×480，步长为 640×3，并可通过 CvBridge 解码回 640×480×3 的 `uint8` 图像。

### 4.2 表面异常图

样本由现有 `tank_vision.synthetic.rust_anomaly_image()` 生成，并通过真实 classical 检测路径处理。

通过条件及结果：

- 收到四类输出各 1 条；
- 四条消息 Header 均与输入一致；
- 至少收到一个缺陷候选；
- 每个候选的 `class_name == "surface_anomaly"`；
- `backend_name == "classical_surface_anomaly"`；
- 置信度有限且在 [0, 1]；
- 面积为有限正值且不超过图像面积；
- `bbox_xyxy` 为合法图像坐标，满足 `0 <= x1 < x2 <= width`、`0 <= y1 < y2 <= height`；
- 质心和多边形坐标均在图像范围内；
- 内部 `failed_frames == 0`、classical detector 无 warning。

该结果只证明合成场景和 ROS 传输的一致性，不证明真实表面缺陷检测精度。 `surface_anomaly` 是通用异常候选类别，不等价于已确认的腐蚀、裂纹、凹坑或其他业务缺陷。

### 4.3 多帧关联

测试发送三张内容不同的图像，每张使用不同的时间戳和 `frame_id`：

```text
synthetic_normal_a
synthetic_anomaly_b
synthetic_normal_c
```

三帧在有界、受控数量内连续发布，没有进行队列过载或压力测试。验证结果：

- 四个输出话题各收到 3 条消息；
- 每个话题的 Header key 集合都恰好等于三组输入 `(sec, nanosec, frame_id)`；
- 每个输入 key 在每个输出话题中只出现一次；
- `synthetic_anomaly_b` 对应的 `DefectArray` 非空；
- 两张正常图对应的 `DefectArray` 为空；
- 三组 debug image 均保持 `bgr8` 和 640×480，并可解码；
- 所有 seam、quality、defect、debug 消息均逐条校验 Header。

因此，本轮在三个受控输入帧范围内验证了检测输出没有错误绑定到后一帧。该验证不覆盖 `drop_oldest`、长时间运行或压力条件。

## 5. 输出消息计数与 Header 校验

每次有效运行的外部消息计数如下：

| 场景 | seam | quality | defects | debug image |
|---|---:|---:|---:|---:|
| 正常焊缝图 | 1 | 1 | 1 | 1 |
| 表面异常图 | 1 | 1 | 1 | 1 |
| 三帧关联 | 3 | 3 | 3 | 3 |
| 合计 | 5 | 5 | 5 | 5 |

每条输出均检查：

```text
message.header.stamp.sec
message.header.stamp.nanosec
message.header.frame_id
```

debug image 另外检查编码、宽度、高度、step、解码结果和 `uint8` 类型。

## 6. 实际运行记录

使用系统 Python、Humble/TROS、新安装目录和独立 ROS domain：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
export TANK_VISION_RUN_ROS2_E2E=1
export PYTHONDONTWRITEBYTECODE=1
```

所有运行均使用总超时 120 秒：

```bash
timeout 120s bash -c '... /usr/bin/python3 -m pytest -q tests/test_ros2_integration.py \
  --junitxml=/home/sunrise/east_eleair/tank_vision_inspection/output/<独立目录>/ros_visual_integration.junit.xml'
```

有效运行结果：

| 运行 | JUnit | 日志 | 结果 |
|---|---|---|---|
| run3 | `output/s0_c_ros_e2e_2026-09-03_run3/ros_visual_integration.junit.xml` | 终端输出未单独持久化 | 3 passed，1.47 秒 |
| run4 | `output/s0_c_ros_e2e_2026-09-03_run4/ros_visual_integration.junit.xml` | 终端输出未单独持久化 | 3 passed，1.47 秒 |
| run5 | `output/s0_c_ros_e2e_2026-09-03_run5/ros_visual_integration.junit.xml` | `output/s0_c_ros_e2e_2026-09-03_run5/pytest.log` | 3 passed，1.47 秒 |

run5 是为持久化标准输出而进行的重复运行；不覆盖 run3 或 run4。

前两次失败尝试也保留：

| 运行 | 结果 | 原因 |
|---|---|---|
| run1 | 3 failed | `rclpy.init()` 默认尝试写入不可写的 `/home/sunrise/.ros/log` |
| run2 | 3 failed | 测试使用了 Humble `Subscription` 不提供的 `get_publisher_count()` |

上述问题均发生在测试辅助层，已在 `tests/test_ros2_integration.py` 修正；生产算法和节点未修改。启用 E2E 的有效运行没有跳过项。

## 7. 资源清理

每个测试使用独立 pytest 临时目录、临时配置、消息缓存和 ROS 节点上下文。

在 `try/finally` 中完成：

- 移除测试节点和视觉节点；
- 销毁测试节点和 `TankVisionNode`；
- `TankVisionNode.destroy_node()` 关闭 inspection/realtime pipeline；
- 关闭 executor；
- 调用 `rclpy.shutdown()`；
- 恢复原 `ROS_LOG_DIR`。

未停止或清理测试之外的进程，未发布运动话题，未访问真实设备。

## 8. 已知风险（本轮不修复）

1. `drop_oldest` 淘汰已接受帧时，节点的 `_inspection_headers` 可能残留。
2. 检测失败警告没有通过当前 `DefectArray` 完整传出，外部订阅者无法仅凭空数组确认检测成功。
3. ROS 图像回调使用 `realtime_pipeline.process()`，不是实时最新帧工作队列。
4. 本轮不做队列过载、长时间内存增长或真实 15 FPS 验收。
5. 本轮三帧关联是有界受控发送，不代表压力下的丢弃、乱序或资源增长行为。

## 9. 复核命令和复现边界

### 9.1 干净 Humble/TROS 和新安装目录

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection

env -i HOME=/home/sunrise USER="$USER" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    source /opt/ros/humble/setup.bash
    source /opt/tros/humble/setup.bash
    source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
    export ROS_DOMAIN_ID=83
    export ROS_LOCALHOST_ONLY=1
    export TANK_VISION_RUN_ROS2_E2E=1
    export PYTHONDONTWRITEBYTECODE=1
    /usr/bin/python3 -m pytest -q tests/test_ros2_integration.py \
      --junitxml=/home/sunrise/east_eleair/tank_vision_inspection/output/s0_c_ros_e2e_recheck/ros_visual_integration.junit.xml
  '
```

该命令会生成新的 JUnit 输出，并要求当前源码、安装目录、Python 依赖、ROS/TROS 环境和 DDS 配置保持兼容，不能表述为已保证完全复现。

### 9.2 不要使用的环境

- 不要加载旧的 `ros2_ws/install/setup.bash`；
- 不要使用旧机器路径 `/home/dts/visoin_track/tank_vision_inspection`；
- 不要加载 Jazzy；
- 不要将本测试当作真实相机或 MCAP 测试；
- 不要在本测试中启动相机、雷达、BPU、Rerun 或机器人控制程序。

## 10. Git 和证据边界

本轮未执行 Git 写操作。工作树仍未提交，且保留本轮开始前已有修改和未跟踪内容，包括相机接入、相机配置、Rerun 源码、采集证据和独立构建产物。

本轮新增/修改的工程文件只有：

```text
tests/test_ros2_integration.py
docs/S0_C_ROS_VISUAL_INTEGRATION.md
```

运行产生的 `output/s0_c_ros_e2e_2026-09-03_run1` 至 `run5` 是独立测试证据目录，不覆盖 S0-B 实验目录、S0-B JUnit 或既有哈希证据。

## 11. 明确非结论

本轮验证不声称：

- 完成真实 MindVision 采集；
- 完成 MCAP 录制或回放；
- 完成相机—雷达硬件同步；
- 完成相机标定或毫米测量精度验收；
- 完成 BPU 模型加载、推理或性能验收；
- 完成真实圆柱资产 BEV；
- 完成机器人运动或底盘融合；
- 完成队列过载、长时间稳定性或正式生产 P95 验收。
