# FIELD-W1 单焊缝视觉现场部署

本阶段只验收 4096×3072 MindVision 的中央区域像素级焊缝巡迹。`StableSeamNode` 使用
`ros_topic`，相机唯一拥有者是 `mindvision_camera_driver`；不启动
`stable_seam_direct`、FAST-LIVO2、最终 BEV、缺陷模型或毫米换算。

## 统一数据流

```text
MindVision camera/front
  -> /camera/front/image_raw + /camera/front/camera_info
  -> StableSeamNode(ros_topic)
  -> /vision/front/seam_geometry + /vision/front/seam_state
  -> /diagnostics + live_session + sparse failure snapshots
  -> MCAP -> validate_bag --visual-profile -> finalize_bag -> SHA
```

`CameraInfo` 当前预期为未标定零 K/D/P。它只产生告警；本阶段不允许以它做投影、BEV
或毫米测量。rough extrinsics 也不启用。

## 预检与启动

在已 source Humble、`physic_bev/install` 和视觉工作区 install 后：

```bash
cd /home/sunrise/east_eleair/physic_bev
export FIELD_DATA_ROOT=/media/mcap_data
export FIELD_CAMERA_SERIAL=sn_XXXXXXXXXXXX
export FIELD_ALLOW_HARDWARE_PENDING=1  # 仅实验室无相机时使用
scripts/field_visual_preflight.py \
  --data-root "$FIELD_DATA_ROOT" --run-id preflight_only \
  --tracker-config ../tank_vision_inspection/configs/s100p_guided_anchor_seam_4096x3072.yaml \
  --tracker-reference ../tank_vision_inspection/configs/seam_geometry_reference_s100p_live_v1.json
```

现场启动：

```bash
scripts/start_field_visual.sh \
  record_raw_image:=false publish_debug_image:=false \
  exposure_mode:=auto exposure_us:=0.0 \
  ae_min_exposure_us:=100.0 ae_max_exposure_us:=50000.0 \
  ae_min_analog_gain_raw:=2 ae_max_analog_gain_raw:=16
```

需要短时原始图像 MCAP 时使用 `record_raw_image:=true`；4096×3072 BGR8 的实际吞吐必须
由闭包后的 MCAP 报告确认，不得以估算代替。长时模式只录 geometry/state/diagnostics、
CameraInfo 和 tf_static，失败 recorder 保存代表性帧。

手动快照：

```bash
ros2 service call /stable_seam_node/save_snapshot std_srvs/srv/Trigger '{}'
```

## 实时调参 UI

先启动唯一相机拥有者和 `StableSeamNode`，并将 `publish_debug_image:=true`；随后另开终端：

```bash
cd /home/sunrise/east_eleair
source /opt/ros/humble/setup.bash
source physic_bev/install/setup.bash
source tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
python3 physic_bev/scripts/seam_tuning_ui.py \
  --config tank_vision_inspection/configs/s100p_guided_anchor_seam_4096x3072.yaml
```

UI只发送当前节点的 `tune_*` 运行时参数，不修改 YAML、reference 或默认 stable 配置；滑块更新会
实时反馈 debug 图、detected/state、coverage、残差、处理耗时和失败原因。关闭UI不会改变配置文件。
可调参数范围由节点校验，非法值会被拒绝。正式验收时应关闭UI和debug输出，避免调参结果混入正式run。

停止、校验、固化：

```bash
scripts/stop_field_visual.sh RUN_ID
```

停止顺序是 launch SIGINT → 等待 rosbag 关闭 → `ros2 bag info` → `validate_bag` →
`finalize_bag` → run manifest/SHA。校验失败的 run 保留原始数据，并标记
`valid_for_formal_validation=false`，不会 finalize。

## 证据目录

```text
DATA_ROOT/runs/RUN_ID/
├── mcap/
├── live_session/
├── failure_snapshots/
├── logs/
├── frozen_configs/
├── run_manifest.json
├── preflight_report.json / .txt
└── field_notes.md
```

## 硬件验收命令

相机不启动第二个拥有者，先由现场 launch 启动驱动，再执行：

```bash
scripts/test_mindvision_1000_frames.py \
  --output /media/mcap_data/runs/RUN_ID/logs/camera_1000_frames.json
```

该脚本只订阅图像，不启动相机。曝光、analog gain/raw、`gain_db` 可用性和读帧耗时来自
`/diagnostics`；SDK 没有的 `gain_db` 会保持 unavailable。MCAP 收尾后：

```bash
scripts/measure_visual_mcap_throughput.py /media/mcap_data/runs/RUN_ID/mcap --input-hz 10
```

只有真实硬件跑完后才能填写相机和吞吐数字；无相机、MID-360 或 DDS 时报告为
`hardware validation pending`。

## 当前边界

- guided-anchor 是 field-validation candidate，不改变 stable 默认配置。
- `/camera/front/camera_info` 仍是不标定 CameraInfo。
- 不启用 rough extrinsics 物理投影，不启用 software clock match。
- MID-360 只有在视觉闭环通过后才用 `start_mid360:=true` 单独确认 lidar/imu 数据和退出行为。
