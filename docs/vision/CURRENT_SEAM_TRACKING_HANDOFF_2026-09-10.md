# CURRENT SEAM TRACKING HANDOFF 2026-09-10

审计编号：S1-SYNC-20260910。本文只记录当前文件系统可复核的代码、配置和既有产物；本轮没有运行相机、ROS、BPU、现场采集、构建、参数搜索或消融实验。

## 1. 一句话当前结论

当前默认 direct 仍运行 stable 配置，不是 guided；guided 在冻结 P2C 现场失败帧上能 `detected=true` 并三帧进入 `TRACKING`，但 center P95=44.15 px、left/right P95=59.28/47.92 px、width P95=70.42 px，严格精度 gate 未通过。当前唯一优先根因假设是左右候选已有但 pair unary 对边界身份判别不足，DP 又把局部错误串成连续路径。

## 2. 当前目标

本阶段目标是建立 ChatGPT/Codex 共用的只读基线：默认运行入口、真实参数、权威输入、可用实验结果、失效历史结论和一个下一步突破点。没有把 `detected=true` 当成精度成功，也没有用降低 gate 的方式解释结果。

## 3. 硬件与运行环境

- 当前主机证据：aarch64、PREEMPT_RT Linux `6.1.158-rt58...`，Python 3.10.12，CMake 3.22.1。
- MindVision 现场产物记录的设备序列号为 `046081820650`；稳定生产现场验证使用 4096×3072、BGR8、resolution index 0、自动曝光、warmup 60 帧。
- 另有 1920×1200 相机链路产物，但固定-FOV tracker 明确拒绝不同宽高比，不能当作 4096×3072 精度证据。
- 已有 4096×3072 稳定现场 300 帧产物：duration 42.69 s，processing P50=77.49 ms、P95=92.59 ms，detected rate=86/300=28.67%；这是稳定配置现场行为，不是 guided 精度验收。

## 4. 当前真实运行入口

实际调用关系为：

```text
stable_seam_direct.launch.py
  -> tank_vision_ros/stable_seam_node
  -> StableSeamNode._direct_worker
  -> MindVisionCameraSource.__next__  [相机读取]
  -> StableSeamTracker.process       [同一 direct worker 串行处理]
  -> SeamGeometry /vision/front/seam_geometry
     SeamState /vision/front/seam_state
     optional Image /vision/front/debug_image
     Trigger /stable_seam_node/save_snapshot
```

`stable_seam_direct.launch.py` 只传入 `input_mode=mindvision_direct` 和 `publish_debug_image=false`，不传 `config_file` 或 `reference_file`。节点默认实际加载安装空间的：

- `ros2_ws/install_s100p_humble/tank_vision_ros/share/tank_vision_ros/config/s100p_stable_seam_4096x3072.yaml`
- `ros2_ws/install_s100p_humble/tank_vision_ros/share/tank_vision_ros/config/seam_width_reference_s100p_v1.json`

这两个安装文件与 source SHA256 一致；因此当前默认 direct 是原 stable 生产配置。`ros2 launch ... --show-args` 显示 `No arguments` 的原因是 launch 文件没有 `DeclareLaunchArgument`，没有任何可覆盖的 launch 参数。

源码和 install 中都不存在 `stable_seam_direct_guided.launch.py`；guided YAML 和 live geometry reference 只在 source 存在，均未安装，不能作为独立 ROS 现场入口。没有执行构建来“补齐”它们。

## 5. 当前算法结构和参数

真实生产候选在 `tank_vision/stable_seam_tracker.py`，配置在两个 YAML 中。输入必须是 4096×3072；`same_fov_scaled=false`，不同宽高比直接拒绝。

- Stable ROI：ratio x=0.30..0.60、y=0.40..0.98，即 `[1229,1229,1229,1782]`；guided/ROI-only 实验 ROI：y=0.50..0.92，即 `[1229,1536,1229,1290]`。
- ROI 处理最大宽度 896；Gaussian 5、Sobel 3、profile Gaussian 9。scan band 半高 4 processed px、步长 8 processed px；stable 现场实例 162 带，guided/ROI-only 实例 117 带。
- signed-edge：Sobel x；左边界取正响应，右边界取负响应。每行阈值为 `max(0.8, abs(response) 的 88 分位数)`；局部极值间隔 5 px，边缘 margin 为 `max(2, 0.025*processed_width)`。全局每侧 Top-K=12。
- guided local search 未进入默认生产类，但在 `StableSeamTracker._generate_candidates()` 中已被代码接入：半径 `0.40 * expected_width`，每侧保留 3 个，空侧才按 `fallback_to_global_search=true` 回退。P2D 117 带中左右均为 local，未发生 fallback。
- pair 硬宽度范围 `0.55*w_ref..1.80*w_ref`，每带最多 8 个 pair。unary 权重依次为：left edge 0.10、right edge 0.10、edge balance 0.12、contrast 0.18、width reference 0.24、center prior 0.10、temporal center 0.09、temporal width 0.07。
- DP：center/width/left/right transition 权重为 0.42/0.24/0.12/0.12，skip penalty 0.38，最多连续缺失 3 带；center jump 0.80×宽度、boundary jump 1.10×宽度、width log-ratio 0.55。
- 拟合：center 为 y→x 二次多项式，width 为一次多项式；Huber IRLS 8 次，center delta 20 px，width delta scale 2。joint inlier 是 center 和 width 两套 inlier 的交集，残差为中心残差与半宽残差的平方和开方。
- geometry gate：点数≥20、coverage≥0.60、effective span≥0.65、最长缺失≤3、突跳数≤3、joint inlier≥0.68、fit residual≤14 px、path confidence≥0.42，且拟合宽度为正并留在 ROI 内。
- temporal：acquire 3 帧、degraded 最多 2 帧、连续失败 3 帧 LOST、EMA alpha 0.50；lateral jump 0.80×旧宽度、width change 40%、heading change 8°、预测置信度每失败帧乘 0.50。失败时输出 `detected=false`，而不是把预测伪装成当前检测。
- debug image 只有参数打开且有订阅者时生成。snapshot 服务最多接受 4 个请求，task queue 为 4；处理完成后复制同帧 header、raw、result、geometry 进入 writer 线程，metadata 最后写入，`complete=true` 才表示目录可消费。

默认 stable 参数、guided 实验参数、历史诊断参数的边界：stable YAML 是当前 direct 生产候选；guided YAML 仅实验；D1-D12 的旧 detector/诊断参数不是当前 tracker 参数，且依赖旧标注的精度结论不能复用。

## 6. 权威输入与 ground truth

P1 冻结参考仍有效：原图 `seam_center_lit_quality_01.bmp` SHA256=`f49df2...9bbaf92`，V2 JSON SHA256=`f960bf...3a68c6`，V2 overlay SHA256=`3d86bb...b02542`。manifest 为 `frozen`，人工 overlay 已批准，JSON/overlay 像素一致，左右点数 18/24；它是冻结参考/合成验证证据，不是 P2C 现场失败帧。

当前现场失败快照目录为 `output/stable_seam_failure_snapshots/20260909_120617_048_stamp_7764_693100000/`，`metadata.complete=true`。raw.bmp SHA256=`ada05bfa...baccd02`，与已知哈希及 metadata 一致；overlay、roi_raw、seam_geometry 也与 metadata 一致。roi_overlay.png 存在，但 metadata 没有对应 hash 字段，故不能声称“被 metadata 覆盖”。快照状态是 `detected=false/LOST`，失败原因 `inlier_ratio_below_minimum`。

P2C 权威现场 ground truth 是 `output/s1_p2c_live_bad_ground_truth_stamp_7764_693100000/raw_annotation.json`，冻结副本为 `output/s1_p2c_live_bad_ground_truth_v1/raw_annotation.json`；两者 SHA256=`c43ab7...4010a15`，左右各 9 点、共 18 点，输入图 SHA 与 raw.bmp 一致。冻结 manifest 为 `status=frozen` 且 `human_overlay_approved=true`；标注 overlay 在两个目录 byte-identical。不得和早期 D7/D8 标注混用。

旧标注 SHA=`1e0bc8...3e7b21` 的原文件当前找不到。S1-D12R integrity audit 已明确 `old_annotation_not_found`、语义身份无法确定、最终结论无效；因此 S1-D8～D12/D12R 中依赖旧标注的误差、near/far、配对率结论均标为 `invalid_due_to_annotation_change` 或 `incomplete`，不是当前权威结果。

## 7. baseline / ROI-only / guided 对比

以下均来自既有 P2D-SIMPLE JSON/CSV，未重新跑实验；误差 P95 是选中 pair 对 P2C 冻结现场标注的绝对误差。

| variant | config / reference | ROI | path / coverage / span | joint inlier / residual | 第1→第3帧 | center P95 | left P95 | right P95 | width P95 | processing P50/P95 ms | 默认 direct | gate |
|---|---|---|---|---|---|---:|---:|---:|---:|---:|---|---|
| baseline | stable YAML / width reference | 1229,1229,1229,1782 | 161/162, .9938, 1.0 | .5528 / 11.04 | LOST→LOST | 132.05 | 119.08 | 130.57 | 98.65 | 87.26/97.16 | 是 | 否，首因率低于 .68 |
| ROI-only | guided YAML，guided disabled / live geometry reference | 1229,1536,1229,1290 | 117/117, 1.0, 1.0 | .7692→.7949 / 9.36→9.67 | ACQUIRING→TRACKING | 59.32 | 68.72 | 96.23 | 58.24 | 65.38/72.40 | 否 | 否 |
| guided | guided YAML / live geometry reference | 1229,1536,1229,1290 | 116/117, .9915, 1.0 | .7931→.8017 / 10.81→10.36 | ACQUIRING→TRACKING | 44.15 | 59.28 | 47.92 | 70.42 | 67.67/74.65 | 否 | 否，首个精度失败 center P95≤25 |

guided 的 `detected=true`、三帧 TRACKING、joint inlier 和处理时间 gate 通过，但精度 gate 没通过；不能把它升级为生产路径。现有负样本回归中三种变体均未报告 detected false positive；该结果只覆盖既有负样本，不等于现场泛化证明。

P0 的 signed error 只有 baseline `scan_band_diagnostics.csv` 提供了足够原始列，可复核得到（selected−manual，159 带）：left P50/P90/P95/mean=`4.34/76.28/119.08/17.60` px；right=`-8.52/11.10/130.57/-11.21`；center=`-2.36/37.15/132.05/3.20`；width=`-18.30/4.71/9.68/-28.82`。baseline center 绝对误差集中在原图 y=2700–3000（mean 60.25、p95 186.69、max 199.87），其次是 1500–1800 与 2400–2700。guided 逐带 signed-error 没有导出，故其 signed P50/P90/P95/mean 为 unknown，不从绝对误差倒推。

## 8. 已失效或被替代的历史结论

| 阶段 | 状态 | 可信边界 |
|---|---|---|
| S1-B2b | valid_but_experimental | 896 实时性能通过；不是现场精度验收 |
| S1-B2c | incomplete | 负场景完整，阳性证据不足 |
| S1-D1 | superseded | 旧 classical detector 诊断，不是当前 stable tracker |
| S1-D2～D7 | superseded | 历史方向/置信度/标注构建实验，不替代 P2C 冻结证据 |
| S1-D8～D12 | invalid_due_to_annotation_change | 依赖不可复核旧标注 |
| S1-D12R | invalid_due_to_annotation_change | integrity audit 明确旧标注缺失、最终结论无效 |
| S1-P1 | valid_but_experimental | 软件/离线 benchmark 与冻结参考有效；硬件实时仍 pending |
| S1-P2A | hardware_pending | SDK 找不到相机，未生成 live bad case |
| S1-P2B | valid_but_experimental | snapshot writer 和离线 benchmark 已有证据，不是完整 ROS/相机频率证据 |
| S1-P2C | valid_but_experimental | 一张冻结现场失败帧及 18 点标注；guided 精度未过 gate |
| S1-P2D-SIMPLE | valid_but_experimental | 单张 P2C 正样本 + 既有负样本/合成回归；未接入默认 ROS |
| guided direct launch 后续 | incomplete | source/install 均无独立 guided launch |

## 9. 当前三个最重要的问题

1. P0 几何精度：guided 通过几何内部一致性，却没有通过人工边界精度；D9/P2C 证据指向 pair identity/unary 与路径选择。
2. P1 入口一致性：source/install 的 stable direct 路径一致，但 guided YAML/reference 未安装且没有 guided launch；文件名不能代表运行配置。
3. P2/P3 系统证据：direct 是相机读取与 process 串行，所以相机约 50 ms 加算法约 75–80 ms 后现场约 8 Hz 是合理的；同时 guided 还缺多姿态、运动连续帧的现场精度验证。

## 10. 唯一下一步突破任务

只改一个对象：`tank_vision/stable_seam_tracker.py` 的 `StableSeamTracker._generate_candidates()` 中 pair unary score 构造区域，做一个“边界身份判别”实验。A 使用当前 guided YAML/reference，B 只改这一处，固定 ROI、guided 半径/Top-K/fallback、reference、DP、geometry gate、temporal 和 ROS 入口。样本固定为 P2C frozen raw.bmp（SHA=`ada05bfa...baccd02`），A/B 只做一次最小离线对比，并复用既有负样本回归。

成功必须同时满足：center P95≤25、left/right P95≤30、width P95≤30、joint inlier≥0.75、coverage≥0.85、无 crossing、突跳≤2、三帧 TRACKING、负样本零 false positive、processing P95<100 ms。任一项失败即停止，不调第二个参数、不降低 gate、不扩大接受范围，不改 reference，不改 DP，不接 ROS。预算是一个冻结正样本、一个既有负样本集、一次 A/B 和一次性能检查。

这个选择优先于“内部纹理边缘”或“reference 模型偏差”：现有 `reference_vs_manual_width.json` 的 `narrow_internal_edge_evidence=false`，而 D9/P2C 已直接记录正确 pair 低 unary 分数、候选存在但 pair 未形成、以及左右单侧选错。因此这是当前证据最强且最小的单点突破；若该 A/B 仍不能同时过 gate，下一阶段才转为只读逐扫描带诊断，不继续猜参数。

## 11. 明确禁止事项

本基线阶段禁止：继续实现算法；改算法/YAML/reference/ROS/message/test/原图/标注/历史输出；启动相机、ROS、BPU 或现场采集；重新 colcon build；参数搜索/消融；降低 geometry gate 或精度门槛；直接改 install 副本；git clean/reset/checkout/restore/stash/commit。

## 12. 复现和现场验证命令

以下是后续复核命令，不是本次执行记录：

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection
sha256sum output/s1_p2c_live_bad_ground_truth_v1/raw.bmp
python3 scripts/run_stable_seam_tracker.py output/s1_p2c_live_bad_ground_truth_v1/raw.bmp --config configs/s100p_stable_seam_4096x3072.yaml --output /tmp/s1_sync_baseline --no-display --save-json
python3 scripts/run_s1_p2d_simple_guided_search.py
```

最后一条会重新产生实验输出，当前审计没有执行。ROS/相机现场命令必须在几何 gate 通过、用户明确授权并完成单独硬件窗口后执行；本报告不把它们当作本次复现步骤。

## 13. Git 状态及 source/install 一致性

Git 根目录是 `/home/sunrise/east_eleair/tank_vision_inspection`，HEAD 为 `33e59417a17e764a127c7d91be1b5098f9a348ba`。审计开始时 `git status --short --untracked-files=all` 有 20 个 modified、1449 个 untracked 条目；其中大量是既有 build/install/log/output 和一般 inspection 改动，均未触碰。焊缝巡迹 source、配置、launch、node、snapshot writer 的 default direct 可比较文件与 install SHA256 一致；guided YAML、live geometry reference、guided launch 在 install 中不存在。没有因为 install 状态执行构建。

本次只新建本交接报告和 `output/s1_sync_current_state_20260910/` 下的审计 JSON；现有文件、安装副本、原图、标注和历史实验输出均未修改。
