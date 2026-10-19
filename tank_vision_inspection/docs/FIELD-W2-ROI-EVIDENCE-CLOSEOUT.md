# FIELD-W2-ROI-EVIDENCE-CLOSEOUT

## Scope

本阶段只补齐现场证据和验收口径，不修改焊缝算法、ROI、reference、pair/unary、DP、拟合、geometry gate 或 temporal 参数。

当前固定证据：

- config SHA256：`6acf482a6cb42b8fff65c8067f4ab298b24e653d4e0453e324b79078550ba438`
- reference SHA256：`0324547cf7d7d304b16deca1e06550c3c212fd1615c21caab5f6c4a49bde7935`
- A01 common-window：664 processed frames，detected 99.55%，TRACKING 98.49%，LOST 0；三个低 joint-inlier 帧被 DEGRADED 吸收。
- 现有严格 A01 结果保持原样，不被 operational 结果覆盖。

## 只读审计结论

三个低内点事件目录位于：

```text
/media/mcap_data/runs/FIELD-W1-HW-CLOSEOUT_A01_RERUN_20260917_131950/live_session/20260917_131952/failure_snapshots/
```

指定事件：

```text
event_1789622403656463446_c9565691
event_1789622413465717999_f112647f
event_1789622413745227338_b9a73882
```

三者的 `failure_reason.json` 都显示：

```text
failure_reasons=["inlier_ratio_below_minimum"]
coverage=1.0
effective_span=1.0
fit_residual_px <= 14
positive_width=true
inside_roi=true
```

因此本次低内点证据不能被推断为光照问题或 SSD 问题。

当前录制配置审计：

- `qos_overrides.yaml`：MindVision image/camera_info 为 `best_effort`、volatile；`
- `mcap_writer_options.yaml`：MCAP、Zstd Fast、64 MiB chunk；
- 全分辨率 BGR8 raw 不作为连续现场默认录制方案。

## Crossing 字段定义

每个已处理帧新增以下字段，均为 O(N) 轻量统计，不开启完整 provenance：

| 字段 | 定义 |
| --- | --- |
| `selected_pair_crossing_count` | DP 最终 selected pair 中满足 `left >= right` 的 pair 数量。正常 pair 强制 `left < right`，预期为 0。 |
| `fitted_output_crossing_count` | 最终输出左右边界逐采样点中满足 `left_x >= right_x` 的点数量。适用于有效输出和 DEGRADED 预测输出。 |
| `minimum_fitted_width_px` | 最终输出宽度 profile 的最小像素宽度；没有输出 profile 时为 `null`。 |
| `crossing_count` | 兼容字段，等于 `fitted_output_crossing_count`。 |

现有 pair 左右顺序约束和拟合正宽检查保持不变；这些字段只记录结果，不改变算法决策。

## ROI validation recorder

StableSeamNode 新增参数，默认关闭：

```text
roi_validation_recording_enabled=false
roi_validation_output_root=/media/mcap_data/runs/roi_validation
roi_validation_queue_size=8
roi_validation_max_duration_sec=0.0
```

开启后，每个节点 session 生成：

```text
ROI_SESSION/
├── roi_frames.raw
├── frames.jsonl
├── manifest.json
└── complete.json
```

规则：

- 只记录 StableSeamNode 实际处理的帧，不代表全部相机帧；
- 不新增 ROS image subscriber；
- 首帧冻结 `roi_original_px`，后续 ROI 改变则 `complete=false`；
- `roi_frames.raw` 是连续无损 BGR `uint8` ROI 字节流；
- `frames.jsonl` 包含 `offset`、`length`、`shape`、`dtype`、header stamp、captured/processed index、config/reference SHA、state、detected、failure reasons、geometry、stage timings 和 crossing 字段；
- 处理线程只做 ROI copy 和有界队列 `put_nowait`，磁盘写入由 `stable-seam-roi-writer` 完成；
- 队列满、写入异常、offset 不连续、输入不是 BGR8、ROI 改变或 SHA 失败都会使 `complete=false`；
- `complete.json` 在正常 close 的最后写入；
- recorder 关闭时不做 ROI copy，不做 recorder 磁盘 I/O。

## Strict 与 operational 验收

### Strict acceptance

使用原有 `evaluate_a01.py`。它保留 minimum inlier 每帧门槛和 processing P95 100 ms 门槛，历史 A01 结果不覆盖：

```bash
cd /home/sunrise/east_eleair/physic_bev
source /opt/ros/humble/setup.bash
source /home/sunrise/east_eleair/physic_bev/install/setup.bash

python3 scripts/evaluate_a01.py /media/mcap_data/runs/FIELD-W1-HW-CLOSEOUT_A01_RERUN_20260917_131950
```

### Operational field-validation acceptance

使用新增脚本：

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection
python3 scripts/evaluate_operational_field_validation.py \
  /media/mcap_data/runs/FIELD-W1-HW-CLOSEOUT_A01_RERUN_20260917_131950
```

该脚本输出 `operational_field_validation_acceptance.json`，阈值为：

```text
detected_rate >= 0.95
tracking_rate >= 0.95
enter_tracking <= 3 processed frames
LOST_after_acquisition = 0
geometry_pass_rate >= 0.99
maximum_consecutive_predicted_only <= 2
coverage_min >= 0.60
residual_max <= 14 px
fitted_output_crossing_count = 0
dropped_processed_frames = 0
output_rate >= 6 Hz
processing_p95 <= 120 ms
end_to_end_p95 <= 180 ms
```

Operational geometry pass 明确排除“孤立的 `inlier_ratio_below_minimum`”作为整段运行失败；minimum inlier 仍然是 tracker 的单帧 geometry gate，不能被降低。

## ROI replay

ROI recorder session 关闭后执行：

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection
python3 scripts/replay_roi_validation.py \
  /media/mcap_data/runs/RUN_ID/roi_validation/YYYYMMDD_HHMMSS
```

输出：

```text
replay_report.json
```

回放从 cold-start tracker 开始，将 ROI raw 重建到冻结的 full-shape 图像，使用同一 config/reference SHA，逐帧比较 state、detected、failure reasons、geometry、crossing 和独立 deterministic tracker 结果。

`mismatch_count=0` 只表示相同冻结 ROI 序列、相同配置/reference、相同初始状态下的确定性回放；不外推到缺帧或不同 temporal 初始状态的序列。

## recorder 关闭/开启性能对比

关闭 recorder 的基线运行：

```bash
ros2 launch tank_vision_ros stable_seam_topic.launch.py \
  image_topic:=/camera/front/image_raw \
  camera_info_topic:=/camera/front/camera_info \
  config_file:=/home/sunrise/east_eleair/tank_vision_inspection/configs/s100p_guided_anchor_seam_4096x3072.yaml \
  reference_file:=/home/sunrise/east_eleair/tank_vision_inspection/configs/seam_geometry_reference_s100p_live_v1.json \
  publish_debug_image:=false \
  roi_validation_recording_enabled:=false
```

开启 ROI recorder：

```bash
ros2 launch tank_vision_ros stable_seam_topic.launch.py \
  image_topic:=/camera/front/image_raw \
  camera_info_topic:=/camera/front/camera_info \
  config_file:=/home/sunrise/east_eleair/tank_vision_inspection/configs/s100p_guided_anchor_seam_4096x3072.yaml \
  reference_file:=/home/sunrise/east_eleair/tank_vision_inspection/configs/seam_geometry_reference_s100p_live_v1.json \
  publish_debug_image:=false \
  roi_validation_recording_enabled:=true \
  roi_validation_output_root:=/media/mcap_data/runs/ROI_PERF_COMPARE \
  roi_validation_queue_size:=8
```

两次运行使用相同输入窗口，比较各自 `live_summary.json` 的 `processing_ms.p95`、output rate、dropped 和 recorder complete 状态。recorder 开启时 processing P95 不能比关闭时回退超过 5%。

## 现场 10 分钟预试

先完成源码的临时构建，避免直接改写现有 install：

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection
source /opt/ros/humble/setup.bash
mkdir -p /tmp/field_w2_tree/tank_vision_inspection/ros2_ws
ln -sfn /home/sunrise/east_eleair/tank_vision_inspection/configs \
  /tmp/field_w2_tree/tank_vision_inspection/configs
colcon build --symlink-install \
  --build-base /tmp/field_w2_tree/tank_vision_inspection/ros2_ws/build \
  --install-base /tmp/field_w2_tree/tank_vision_inspection/ros2_ws/install

cd /home/sunrise/east_eleair/physic_bev
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --build-base /tmp/field_w2_tree/physic_bev/build \
  --install-base /tmp/field_w2_tree/physic_bev/install \
  --packages-up-to inspection_interfaces mindvision_camera_driver inspection_tools inspection_bringup
```

预试启动：

```bash
cd /home/sunrise/east_eleair/physic_bev
source /opt/ros/humble/setup.bash
source /tmp/field_w2_tree/tank_vision_inspection/ros2_ws/install/setup.bash
source /tmp/field_w2_tree/physic_bev/install/setup.bash

export FIELD_DATA_ROOT=/media/mcap_data
export FIELD_RUN_ID=FIELD-W2-ROI-PRE10-$(date +%Y%m%d_%H%M%S)
export FIELD_CAMERA_SERIAL=sn_046081820650
export FIELD_PHYSIC_INSTALL=/tmp/field_w2_tree/physic_bev/install
export FIELD_TANK_INSTALL=/tmp/field_w2_tree/tank_vision_inspection/ros2_ws/install

./scripts/start_field_visual.sh \
  publish_debug_image:=false \
  record_mcap:=true \
  record_raw_image:=false \
  auto_save_failures:=true \
  roi_validation_recording_enabled:=true \
  roi_validation_queue_size:=8 \
  roi_validation_max_duration_sec:=600 \
  start_mid360:=false \
  start_sync:=false
```

停止并固化：

```bash
./scripts/stop_field_visual.sh "$FIELD_RUN_ID"
```

然后执行 ROI replay 和 operational acceptance。若任何校验失败，保留整个 run，不删除、不覆盖。

## 现场 30 分钟稳定性

使用同样的临时安装和环境，运行：

```bash
export FIELD_DATA_ROOT=/media/mcap_data
export FIELD_RUN_ID=FIELD-W2-ROI-STABLE30-$(date +%Y%m%d_%H%M%S)
export FIELD_CAMERA_SERIAL=sn_046081820650
export FIELD_PHYSIC_INSTALL=/tmp/field_w2_tree/physic_bev/install
export FIELD_TANK_INSTALL=/tmp/field_w2_tree/tank_vision_inspection/ros2_ws/install

./scripts/start_field_visual.sh \
  publish_debug_image:=false \
  record_mcap:=true \
  record_raw_image:=false \
  auto_save_failures:=true \
  roi_validation_recording_enabled:=true \
  roi_validation_queue_size:=8 \
  roi_validation_max_duration_sec:=1800 \
  start_mid360:=false \
  start_sync:=false
```

约 30 分钟后：

```bash
./scripts/stop_field_visual.sh "$FIELD_RUN_ID"
```

要求：无崩溃、无 capture error、无 writer error、节点正常 SIGINT、MCAP 可 validate/finalize、ROI `complete=true`、SSD 空间正常，并完成 operational acceptance 和 replay report。

## 当前实现状态与证据等级

## 本次硬件证据

宿主机审计确认 `/dev/nvme0n1p1`（Lenovo E680 NVMe 1TB、ext4、约930 GiB可用）以 `rw` 挂载到 `/media/mcap_data`，小文件写入/删除成功。Codex容器内看到的 `ro` 是只读绑定，不是宿主机挂载结论。

正式轻量MCAP预试：

```text
/media/mcap_data/runs/FIELD-W2-ROI-EVIDENCE-PRE10-20260917_164238
```

该run已 `validate_bag`、`finalize_bag`、SHA256固化通过：6754 geometry/state、10572 camera_info/diagnostics、1个tf_static，timestamp单调；相机实际 `MV-SUF1205GC#0 / 046081820650 / 4096x3072 / bgr8`，auto曝光运行值约49997 us、analog_gain_raw=16、gain_db unavailable。common active window 1056.87 s，input 6.398 Hz，tracker 6.390 Hz，processing P95 116.68 ms，端到端P95 147.81 ms。

但该run当前画面全程 `detected=0 / LOST`，因此不能作为中央焊缝A01通过证据；operational acceptance报告为 `failed`，失败项是 detected/tracking、geometry、residual 和 9 个latest-slot覆盖。该失败run保留，不能归因于SSD。

ROI recorder硬件预试：

```text
/media/mcap_data/runs/FIELD-W2-ROI-RECORDER-PRETEST-20260917_170740/roi_validation/20260917_170750
```

113帧、537453990 bytes、队列丢帧0、写入错误0、offset连续、`complete=true`；冷启动回放 `matched=113 / unmatched=0 / mismatch=0`。开启recorder时 processing P95 117.43 ms，相比关闭run的116.68 ms增加约0.6%，但该短测tracker输出约5.04 Hz，低于6 Hz operational门槛，需在现场继续优化/复测；不能声称recorder开启后已达到生产性能。

- `StableSeamTracker`：field-validation candidate；当前阶段没有把 guided-anchor 改成默认生产配置；
- ROI recorder：默认关闭，现场开启后只记录 processed ROI，不是完整相机序列；
- operational acceptance：用于现场运行可用性，不替代 strict acceptance；
- CameraInfo K/D/P：仍可能为零，当前只允许像素级 tracking；
- rough extrinsics：未启用物理投影；
- 最终 BEV、缺陷模型、FAST-LIVO2 融合：不属于本阶段完成项；
- 10分钟轻量MCAP证据链已完成，但中央焊缝A01和recorder开启时的6 Hz性能仍未通过；30分钟稳定性因10分钟验收未通过而未执行。当前结论为 `field-validation candidate / not production-ready`。
