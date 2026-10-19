# S1-P3 实时焊缝巡检显示与失败事件记录

本阶段只增加显示、轻量逐帧日志和限频失败事件快照，不改变
`StableSeamTracker`、候选、pair、DP、IRLS、geometry gate、temporal、ROI、reference
或 ROS 消息定义。

## 运行方式

guided-anchor 现场验证候选：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
ros2 launch tank_vision_ros stable_seam_direct_guided_anchor.launch.py \
  publish_debug_image:=true \
  auto_save_failures:=true \
  failure_snapshot_cooldown_sec:=2.0 \
  max_failure_snapshots_per_session:=100
```

查看器：

```bash
QT_QPA_PLATFORM=xcb ros2 run tank_vision_ros seam_debug_viewer
```

按 `S` 请求现有完整手动快照，按 `Q` 或 `ESC` 退出。默认 stable launch
未被替换，仍使用其原有配置和参数。

## 线程与数据

direct 模式由相机采集线程将成功帧放入有界 latest-frame-only 槽，处理线程
独立消费最新帧；处理线程只提交轻量 `frame_metrics.jsonl` 行和稀疏失败事件。
JSONL 与事件图像分别由后台 writer 写入。没有事件时不复制全分辨率图像，
也不生成 JPEG 或 overlay。

每个会话目录位于：

```text
output/stable_seam_live_sessions/YYYYMMDD_HHMMSS/
├── session_manifest.json
├── frame_metrics.jsonl
├── events.jsonl
└── failure_snapshots/
```

自动事件目录包括 `roi_raw.bmp`、`context.jpg`、`overlay.png`、
`seam_geometry.json`、`failure_reason.json` 和最后写入的 `metadata.json`。
只有 `metadata.complete=true` 才视为完整事件。自动事件默认最多 100 个，
普通重复失败默认冷却 2 秒；队列满、达到上限或剩余空间低于 500 MB 时拒绝
新自动图像，但继续检测和轻量日志。

实时 tracker 明确使用 `diagnostics=False, provenance=False`。因此生产快路径
未暴露的 path confidence、左右独立 inlier 和 crossing 会在 JSONL 中标为
`metrics_unavailable_fields`，不会用推导值冒充真实 trace 数据。

当前 MindVision `FrameData` 合约没有独立整数 `CameraGetAnalogGain` 字段，
所以 `analog_gain_raw` 保持 `null` 并记录不可用原因；浮点 gain 与 raw gain
不混写。
