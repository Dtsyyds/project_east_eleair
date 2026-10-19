# S1-B2b：896 像素实时焊缝处理复测报告

日期：2026-09-07

## 结论

896 像素候选在当前真实 MindVision MV-SUF1205GC ROS 链路上达到本阶段性能门槛：

- 相机设备日志：19.950 fps；
- 相机 Topic 末段观测：17.990 Hz；
- `/vision/seam_state` 末段观测：18.355 Hz；
- 节点最后一个 100 帧统计点：`input_frames=1300`、`real_results=1300`、内部 `dropped_frames=0`；
- `total_algorithm_ms`：mean 42.800 ms、P50 42.281 ms、P95 47.023 ms、max 97.469 ms；
- `seam_detect_ms`：mean 2.376 ms；
- `seam_geometry_ms`：0.000 ms（当前无标注真实画面中未记录到有效几何阶段，不能解释为现场几何精度）；
- 相机 `timeout_count=0`、`capture_error_count=0`；
- 60 秒以上运行期间没有帧处理错误或节点运行时崩溃。

有效日志最后由外层测试超时发送停止信号；视觉节点随后记录了 1 条
`rcl_shutdown already called` 退出路径异常。这是停止信号与 `finally` 双重
shutdown 的测试收尾现象，不是帧处理错误或运行期间崩溃；因此“运行时处理错误”为 0，
“收尾异常”为 1。

因此，本阶段的“seam topic 输出 ≥10 Hz”和“mean total_algorithm_ms <100 ms”通过。该结果是当前主机、当前驱动条件和当前未标注画面的性能结果，不是现场检测精度验收。

## 配置与代码核对

`configs/s100p_realtime_1920x1200_896.yaml` 与 1024 配置的语义化 YAML 差异只有：

```yaml
pipeline:
  thumbnail_max_dimension: 896
```

以下值均保持关闭或不变：

```text
inspection.enabled = false
pipeline.roi = null
pipeline.save_overlay = false
pipeline.save_json = false
pipeline.save_masks = false
```

`seam` 全部参数（包括阈值、二次多项式、RANSAC 120 次和 4.0 px 残差阈值）以及 `laser` 参数在 1024/896 配置中完全相同。原始坐标恢复仍通过 `ImageView.scale_x/scale_y` 完成：横向误差除以处理尺度，中心线通过 `view.to_original()` 恢复到输入图像坐标；ROS 消息定义未改动。

本次开始时 896 配置文件已经存在于工作区且为未跟踪文件；本阶段未覆盖它，只做了上述等价性校验。

## 确定性合成样本对比

使用同一批 1920×1200 合成输入，每套配置重复 4 次，共 24 帧，使用实时焊缝通路直接测量。结果仅用于软件回归：

| 样本 | 1024 detected/confidence | 896 detected/confidence | 1024 lateral / heading (px/deg) | 896 lateral / heading (px/deg) | 中心线点数 1024/896 |
|---|---:|---:|---:|---:|---:|
| straight_center | true / 0.944948 | true / 0.947512 | -0.5185 / 0.0054 | -0.5423 / 0.0034 | 320 / 280 |
| straight_left | true / 0.916118 | true / 0.919880 | -140.2333 / 0.0031 | -140.4220 / 0.0043 | 320 / 280 |
| sloped | true / 0.945931 | true / 0.948490 | 18.7574 / 4.5692 | 18.5796 / 4.5685 | 320 / 280 |
| curved_noisy | true / 0.926704 | true / 0.929297 | 73.8200 / 0.6685 | 73.6965 / 0.6664 | 320 / 280 |
| occluded | true / 0.835797 | true / 0.837556 | -60.5028 / -0.0226 | -60.4241 / -0.0082 | 147 / 129 |
| no_seam | false / 0.000000 | false / 0.000000 | null / null | null / null | 0 / 0 |

| 配置 | total mean/P50/P95/max (ms) | seam_detect mean/P50/P95/max (ms) | seam_geometry mean/P50/P95/max (ms) |
|---|---:|---:|---:|
| 1024 | 96.8099 / 106.9872 / 116.3512 / 116.6808 | 25.5318 / 30.8022 / 33.4491 / 33.6881 | 34.8153 / 42.4508 / 45.8079 / 46.2486 |
| 896 | 92.6930 / 105.2563 / 108.8389 / 109.4015 | 21.9851 / 27.1316 / 28.3382 / 28.6793 | 35.4044 / 43.1783 / 45.1575 / 46.0757 |

合成样本结果只能用于软件回归。当前真实相机画面没有焊缝标注，不能据此声称现场检测精度合格，也不能用两个尺度结果的差异反向修改阈值。本阶段没有修改检测阈值、RANSAC 参数或坐标逻辑。

## 回归与 ROS 验证

新增 `tests/test_s1_b2b_896_profile.py`，覆盖：

- 1024 配置内容未被 896 配置改变；
- inspection、ROI、保存和 debug 开关；
- seam/laser 检测参数及 RANSAC 参数一致；
- 1920×1200 输入下中心线、横向误差和航向误差处于原图坐标；
- debug 参数关闭时不进入 `create_overlay`/CvBridge 路径。

结果：

```text
python3 -m compileall -q tank_vision scripts tests
73 passed, 4 skipped
ROS 2 Humble/TROS E2E: 3 passed in 1.63s
colcon: 4 packages finished
```

ROS 构建使用已有的 `ros2_ws/build_s100p_humble`、`install_s100p_humble` 和 `log_s100p_humble`，没有清理这些目录，也没有修改消息定义、MindVision 驱动或 `physic_bev`。

## dropped_frames 边界

当前 ROS 节点在 `_image_callback()` 中同步调用 `realtime_pipeline.process(frame)`，没有通过 `RealtimeSeamPipeline.submit()` 进入实时队列。因此日志中的 `dropped_frames=0` 只表示该内部 `submit()` 统计没有发生淘汰；它不表示 DDS、相机驱动或 ROS 订阅队列没有丢帧，也不能声称算法处理了全部相机帧。`input_frames` 是节点回调实际收到的帧数，Topic Hz 是订阅者的观测速率。

## 真实复测命令

以下三个终端统一使用 `ROS_DOMAIN_ID=83` 和 `ROS_LOCALHOST_ONLY=1`。驱动使用已有 `physic_bev` 安装，不修改其源码；`ae_max_exposure_us=50000` 是达到约 19.95 fps 的既有条件。

终端 1，相机驱动：

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/east_eleair/physic_bev/install/setup.bash
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
ros2 run mindvision_camera_driver mindvision_camera_node --ros-args \
  -r __ns:=/camera -r __node:=front \
  -p sdk_root:=/home/sunrise/east_eleair/MindVisionlinuxSDK \
  -p serial_number:=sn_046081820650 -p resolution_index:=-1 \
  -p width:=1920 -p height:=1200 -p exposure_mode:=auto -p exposure_us:=0.0 \
  -p ae_min_exposure_us:=100.0 -p ae_max_exposure_us:=50000.0 \
  -p ae_min_analog_gain_raw:=2 -p ae_max_analog_gain_raw:=16 \
  -p warmup_frames:=60 -p timeout_ms:=2000 -p publish_rate_hz:=0.0 \
  -p qos_depth:=1 -p qos_reliability:=best_effort
```

终端 2，视觉节点：

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source ros2_ws/install_s100p_humble/setup.bash
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
ros2 run tank_vision_ros vision_node --ros-args \
  -p config_file:=/home/sunrise/east_eleair/tank_vision_inspection/configs/s100p_realtime_1920x1200_896.yaml \
  -p image_topic:=/camera/front/image_raw \
  -p camera_info_topic:=/camera/front/camera_info \
  -p publish_debug_image:=false
```

终端 3，至少观察 60 秒：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1
ros2 topic hz /camera/front/image_raw
ros2 topic hz /vision/seam_state
```

节点每 100 帧输出一次 `input_frames`、`real_results`、`dropped_frames`、total 和阶段耗时。当前真实画面没有标注，复测只能验收 Topic/吞吐/耗时/稳定性，不能验收检测精度。

## Git 状态

本阶段没有执行 Git commit，没有清理或覆盖已有未提交修改。工作区仍保留开始前的修改和未跟踪文件；本阶段新增的回归测试和本报告也保持未提交。
