---
name: field-w3-system-e2e-debug
description: Continue debugging and validating the frozen MindVision/ROS visual seam field pipeline on the RDK S100P, including preflight, lightweight MCAP, asynchronous evidence, stop/finalize, replay, and system-vs-algorithm acceptance. Use this for FIELD-W3 follow-up work; do not use it to tune the seam algorithm.
---

# FIELD-W3 系统闭环续调

本 skill 用于在 `/home/sunrise/east_eleair` 继续 FIELD-W3 现场视觉闭环工作。它记录今天已经验证过的工程事实和停止条件，不替代当前用户的新任务要求。

## 不可违反的边界

- 保持 StableSeamTracker、candidate、response threshold、Top-K、pair、DP、IRLS、geometry gate、temporal、ROI、reference 和生产 YAML 冻结。
- 不启动 MID-360、FAST-LIVO2、BEV、缺陷模型或物理融合。
- 不修改 install/build 生成物；只改 source。
- 不删除、覆盖历史 run，不执行 `git clean/reset/checkout/commit`。
- 不把算法 LOST 或 detected=false 当作系统录制失败。
- 不把 CameraInfo K/P 为零解释为有效标定；当前只允许像素级跟踪。
- 不录制长时间全分辨率 BGR8 raw MCAP；默认轻量 MCAP。

## 当前已确认事实

工程目录：

```text
/home/sunrise/east_eleair/tank_vision_inspection
/home/sunrise/east_eleair/physic_bev
```

硬件：MindVision MV-SUF1205GC，serial `046081820650`，USB 3.0，4096×3072，BGR8；SSD 为 `/dev/nvme0n1p1`、ext4、label `mcap_data`、挂载 `/media/mcap_data`。

冻结配置 SHA：

```text
guided config: 6acf482a6cb42b8fff65c8067f4ab298b24e653d4e0453e324b79078550ba438
reference:     0324547cf7d7d304b16deca1e06550c3c212fd1615c21caab5f6c4a49bde7935
```

ROS source/install 已通过重建后路径一致性检查：tracker 解析到 `ros2_ws/src/tank_vision_ros/.../stable_seam_node.py`，核心 tracker 解析到 `tank_vision/stable_seam_tracker.py`。正式运行前仍必须重新运行 preflight，不得只相信旧报告。

## 最终硬件证据

主证据 run：

```text
/media/mcap_data/runs/FIELD-W3-E2E-20260917_230533
```

该 run 的 MCAP `validate=true`、`manifest.yaml` 已生成、时间戳单调、正常 SIGINT 停止、无孤儿进程；42/42 failure snapshots 完整且 SHA 正确；provenance 写入 30 个事件，真实 queue drop=0、write error=0，`capacity_suppressed_count=42` 是达到正常证据上限后的有意抑制，不是写入失败。

系统仍未通过的唯一门槛是 common tracker output：

```text
common window: 614.941 s
input:  2034 frames / 3.306 Hz
output: 1955 frames / 3.178 Hz
processing P50/P95/max: 96.32 / 120.17 / 2115.73 ms
required output: >= 6 Hz
```

关闭 provenance 的约 60 秒 smoke 也只有约 `4.42 Hz`，所以低速不是 provenance writer 单独造成的。下一步应先做输入回调、图像转换/source_read、tracker processing 和 latest-frame slot 的分段 profiling；不要先调算法阈值。

算法观察值仅供记录：detected `98.47%`，TRACKING `96.63%`，状态为 LOST 3、ACQUIRING 36、TRACKING 1891、DEGRADED 27。它不构成 production-ready 结论。

## 一键流程

先加载三个环境：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
source /home/sunrise/east_eleair/physic_bev/install/setup.bash
```

启动默认不启用 provenance：

```bash
bash /home/sunrise/east_eleair/tank_vision_inspection/scripts/run_field_visual_e2e.sh
```

需要验证低频 provenance 时显式开启：

```bash
GEOMETRY_PROVENANCE_ENABLED=true \
bash /home/sunrise/east_eleair/tank_vision_inspection/scripts/run_field_visual_e2e.sh
```

正常停止：

```bash
bash /home/sunrise/east_eleair/tank_vision_inspection/scripts/stop_field_visual_e2e.sh RUN_ID
```

脚本会使用 `sn_046081820650` 形式传给 ROS，避免全数字 serial 被 YAML/ROS 参数系统解析成 DOUBLE；驱动内部会剥离 `sn_`，实际序列号不变。

## 续调顺序

1. 重新执行 `field_visual_preflight.py`，确认 SSD 是宿主机 `rw`，MindVision 可枚举，source/install SHA 一致。
2. 只做 60 秒 smoke；重点记录 camera diagnostics、latest-frame slot、processed/dropped、source_read、processing stage 和 RSS。
3. 只有 smoke 的 common output 达到 6 Hz、MCAP validate/finalize 和无孤儿进程全部通过后，才做 10 分钟运行。
4. 低速优化只能先在记录/调度/图像转换/性能 profiling 层进行；若必须改算法数值，停止本 skill 流程并等待用户明确授权。
5. 每个 run 使用新的 `RUN_ID`；validate 失败仍保留 run，禁止 finalize。

## 报告位置与语义

自动报告位于：

```text
tank_vision_inspection/output/field_w3_system_e2e_closeout/
```

重点文件：`system_acceptance.json`、`algorithm_observation.json`、`topic_rate_summary.json`、`timestamp_audit.json`、`recorder_health.json`、`snapshot_integrity.json`、`process_shutdown_audit.json`、`replay_summary.json`、`frozen_software_manifest.json`。

必须分开判断：

- `system_pass`：相机、ROS 节点、消息、MCAP、停止、快照、回放、时间戳和速率门槛。
- `algorithm_pass`：detected、TRACKING、inlier、residual、几何精度等算法指标。

当前结论应保持为：

```text
system_integration_ready=false
algorithm_field_validation_pending=true
production_ready=false
```

## 每次改动后的最低验证

```bash
/usr/bin/python3 -m pytest -q \
  tank_vision_inspection/tests/test_geometry_provenance_recorder.py \
  tank_vision_inspection/tests/test_failure_event_recorder.py \
  physic_bev/src/inspection_bringup/test/test_visual_field_contract.py
/usr/bin/python3 -m compileall -q tank_vision_inspection/tank_vision tank_vision_inspection/scripts
bash -n tank_vision_inspection/scripts/run_field_visual_e2e.sh tank_vision_inspection/scripts/stop_field_visual_e2e.sh
git -C tank_vision_inspection diff --check
```

ROS source 改动后重新 `colcon build/test`，并再次验证 source/install 实际解析路径；不要直接修改 install 副本。
