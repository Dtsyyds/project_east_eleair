# S1-B2c 真实焊缝阳性路径验证

日期：2026-09-07
配置：`configs/s100p_realtime_1920x1200_896.yaml`
实验域：`ROS_DOMAIN_ID=83`，`ROS_LOCALHOST_ONLY=1`
debug image：`publish_debug_image:=false`

## 结论

本阶段新增了只读 ROS 统计工具和纯统计单元测试，完成了 60 秒现场负场景测量，并尝试了阳性场景。负场景链路的输入 Header 单调、SeamState Header 唯一且严格匹配；但现场阳性场景没有得到 `detected=true`，且 `seam_geometry_ms=0`。因此：

**positive-path evidence insufficient**。

本报告不将负场景快速退出性能作为焊缝阳性完整路径性能，也不声称焊缝定位、毫米级误差、缺陷检测率或 BEV 映射精度合格。

## 变更

- 新增 `scripts/validate_ros_seam_realtime.py`。
  - 只订阅 CameraInfo、SeamState、VisionQuality；不订阅 debug image，不使用 `cv_bridge`。
  - CameraInfo 仅作为与 Image 共享 Header 的轻量输入帧证据，不能等同于实际收到的 Image 数量。
  - 按 `(stamp, frame_id)` 做严格 Header 关联；保存有界缓存；输出 `allow_nan=False` 的确定性 JSON。
  - output age 只有在 ROS 接收时钟与 Header stamp 可比较时才计算，否则输出 unavailable 和原因。
- 新增 `tests/test_validate_ros_seam_realtime.py`，覆盖跨秒、重复/非单调 Header、精确匹配、未匹配、detected true/false、None/NaN、空 Topic、JSON 确定性和缓存淘汰。
- 新增 `docs/S1_B2C_REAL_SEAM_VALIDATION_2026-09-07.md`。

既有 1024 配置、896 配置、检测阈值、RANSAC、多项式阶数、坐标恢复、ROS 消息、MindVision 驱动及其他受保护模块未因本阶段测量而修改。

## 现场实验

### A. `negative_no_seam`

证据目录：`output/s1_b2c_negative_no_seam_20260907_rerun2/`

| 指标 | 结果 |
|---|---:|
| duration | 60.000 s |
| device fps | 19.950 |
| CameraInfo count / rate | 1183 / 19.717 Hz |
| SeamState count / rate | 1023 / 17.050 Hz |
| Quality count | 1022 |
| Camera Header 单调 | true |
| Seam Header 单调 | true |
| duplicate Seam stamp | 0 |
| exact Header match | 1023 / 1023 = 1.0 |
| unmatched Seam | 0 |
| seam_detected | 0 / 1023 = 0.0 |
| output age mean / P50 / P95 / max | 52.733 / 51.858 / 58.989 / 179.138 ms |
| invalid quality | 1022 |

负场景每 100 帧日志的最后一个稳定窗口为：`input_frames=1100`、`real_results=1100`、`dropped_frames=0`、total mean/P50/P95/max = `42.472/42.099/46.563/66.503 ms`、`seam_detect_ms=2.105 ms`、`seam_geometry_ms=0 ms`、`output_hz=17.352`。这是无焊缝快速退出路径，不是阳性路径性能。

相机在测量期间报告 `timeout_count=0`、`capture_error_count=0`、device fps `19.950`。测量结束时发送停止信号导致 ROS publisher context invalid 的 teardown 日志；这不是测量窗口内的采集超时或帧处理失败，已在原始日志中保留。

### B. `positive_centered_seam` 尝试

证据目录：`output/s1_b2c_positive_centered_seam_20260907_attempt1/`

按要求先执行了：

```bash
ros2 topic echo --once /vision/front/seam_state
```

但当前 ROS CLI daemon 返回 `RuntimeError: !rclpy.ok()`，未能取得一次 SeamState 样本，因此没有伪造 `detected=true` 的前置确认。随后仍保留了约 60 秒现场尝试，统计为：

| 指标 | 结果 |
|---|---:|
| duration | 60.000 s |
| device fps | 19.950 |
| CameraInfo count / rate | 1141 / 19.017 Hz |
| SeamState count / rate | 1106 / 18.433 Hz |
| Camera Header 单调 | true |
| Seam Header 单调 | true |
| duplicate Seam stamp | 0 |
| exact Header match | 1059 / 1106 = 0.9575 |
| unmatched Seam | 47 |
| seam_detected | 0 / 1106 = 0.0 |
| output age mean / P50 / P95 / max | 51.813 / 50.380 / 62.100 / 150.619 ms |

每 100 帧日志的最终稳定窗口为：`input_frames=1200`、`real_results=1200`、`dropped_frames=0`、total mean/P50/P95/max = `42.579/41.705/49.379/85.536 ms`、`seam_detect_ms=2.138 ms`、`seam_geometry_ms=0 ms`、`output_hz=17.947`。由于检测率为 0，且 geometry 未执行，该轮不能作为阳性完整路径性能证据；阳性验收为证据不足。

相机日志在测量窗口内仍为 device fps `19.950`，`timeout_count=0`、`capture_error_count=0`。同样的停止信号 teardown 日志已保留。

## 工具统计边界

`camera_info_count` 是轻量 CameraInfo Header 证据，不是实际收到的 Image 帧数量。当前 ROS 视觉节点在 callback 中同步执行 `process()`；因此节点内部 `dropped_frames=0` 只表示该同步处理路径的内部队列没有丢弃统计，不能推出 DDS 输入端没有丢帧，也不能声称处理了全部相机帧。

## 软件和 ROS 回归

- `python3 -m compileall -q tank_vision scripts tests`：通过。
- `python3 -m pytest -q`：`77 passed, 4 skipped`。
- ROS E2E：`3 passed`。
- `colcon build --symlink-install`：4 packages finished（此前已完成，未改动 ROS 源后再次运行相关测试）。
- `colcon test`：4 packages finished；各包测试命令完成，ROS 包当前报告 `Ran 0 tests`。

跳过项是既有平台测试在无官方适配器/显式 ROS E2E 开关时的预期跳过，不是 S1-B2c 失败。

## 真实硬件复测命令

三个终端均先执行：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
```

终端 1（相机）：

```bash
source /home/sunrise/east_eleair/physic_bev/install/setup.bash
ros2 run mindvision_camera_driver mindvision_camera_node --ros-args \
  -r __ns:=/camera -r __node:=front \
  -p sdk_root:=/home/sunrise/east_eleair/MindVisionlinuxSDK \
  -p serial_number:=sn_046081820650 \
  -p frame_id:=mindvision_front_optical_frame \
  -p resolution_index:=-1 -p width:=1920 -p height:=1200 \
  -p exposure_mode:=auto -p exposure_us:=0.0 \
  -p ae_min_exposure_us:=100.0 -p ae_max_exposure_us:=50000.0 \
  -p ae_min_analog_gain_raw:=2 -p ae_max_analog_gain_raw:=16 \
  -p warmup_frames:=60 -p timeout_ms:=2000 -p publish_rate_hz:=0.0 \
  -p qos_depth:=1 -p qos_reliability:=best_effort
```

终端 2（视觉节点）：

```bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
ros2 run tank_vision_ros vision_node --ros-args \
  -p config_file:=/home/sunrise/east_eleair/tank_vision_inspection/configs/s100p_realtime_1920x1200_896.yaml \
  -p image_topic:=/camera/front/image_raw \
  -p camera_info_topic:=/camera/front/camera_info \
  -p seam_topic:=/vision/front/seam_state \
  -p quality_topic:=/vision/front/quality \
  -p publish_debug_image:=false
```

终端 3（验证）：

```bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
python3 scripts/validate_ros_seam_realtime.py \
  --duration 60 --scenario positive_centered_seam \
  --camera-info-topic /camera/front/camera_info \
  --seam-topic /vision/front/seam_state \
  --quality-topic /vision/front/quality \
  --output output/s1_b2c_positive_centered_seam_$(date +%Y%m%d_%H%M%S)/validator.json
```

阳性试验开始前另执行一次 `ros2 topic echo --once /vision/front/seam_state`，确认实际字段 `detected: true`。若多数帧不能为 true，必须继续报告 `positive-path evidence insufficient`，不能改阈值迎合结果。

按证据不足停止条件，另保存了一对同 Header 的代表性样本：
`output/s1_b2c_positive_evidence_insufficient_20260907/representative_input.bmp` 和
`representative_seam_state.json`。该 SeamState 为 `detected=false`、
`lost_reason=lost_threshold_reached:no_valid_component`，仅用于后续数据/算法适配，不能作为现场精度结论。

## 有效性边界与 Git

本阶段没有真实标注和相机标定，尚未验证焊缝定位精度、毫米级误差、缺陷检测率或 BEV 映射精度。未启动机器人运动，未接入 BPU，未降低到 768，未修改检测算法。

未执行 Git commit，未清理工作区，既有未提交修改和既有报告均保留。当前工作区仍包含本工程原有的 tracked/untracked 修改；请以 `git status --short` 查看完整状态。
