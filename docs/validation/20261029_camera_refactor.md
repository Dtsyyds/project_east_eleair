# 2026-10-29 三相机重构续作与实测

本轮完成采集可靠性修复、三相机配置、历史诊断重生成和网页录放验证。**三相机持续采集仍未通过硬件验收**：工业相机与鱼眼持续正常，D405 在原分辨率和低分辨率均再次出现 UVC `-71` 后停流。不能把 USB 枚举成功、短时出图、软件测试通过解释为设备稳定可用。

工作树：`/media/mcap_data/project_east_eleair_refactor`。沿用此前未提交修改；本轮未提交或推送。正式 Python 运行代码来自当前 packages 和本工程 `.runtime/s100p/install`；没有使用旧工程业务代码作为运行 fallback。本轮未改 C++ 驱动，沿用前次建立的新工程 SDK/ROS overlay。

## 采集可靠性修复

- 新增 `config/communication/cameras.yaml`，明确要求工业、鱼眼、D405 彩色、深度、左右红外六路。网页标明“三相机采集”，新批次保存方案、必需话题和硬件配置。关闭涡流时桥接配置也标记未配置；`system.yaml` 的原整机必需项保持。
- 数据盘目录消失、stat/disk_usage 报 I/O 错误时，状态查询仍返回明确存储错误，自动停止保护继续执行。
- 修复“每路仅有一帧也被判完整”的漏洞：预检、录制监控、停止前统一检查必需流的 3 秒新鲜度。停止时重新检查，避免录制中断发生在监控周期之间而漏判。流中断自动关闭 writer，批次标记 `incomplete`，保留 MCAP。
- 实机浏览器验收增加六路实际录制帧率断言，不能只看工业相机帧率或其他话题非零。DDS 检查工具增加观察窗口覆盖率、采样时间戳倒退/新鲜度和图像布局检查。

## 实机观测与批次

| 项目 | 结果 |
| --- | --- |
| MindVision 046041920527，1920×1200 | 三次 180 秒观察均持续约 10 Hz；复位后观察 1718 帧，最大接收间隔 0.171 秒 |
| icSpring 鱼眼，640×480 | 三次 180 秒观察均持续约 15 Hz；复位后观察 2653 帧，最大接收间隔 0.181 秒 |
| D405 260322276866，1280×720×30 | 短时出图后停流，180 秒观察未通过 |
| D405，640×480×15 | 普通重启、单设备 USB reset 和临时关闭 autosuspend 后仍停流；复位后观察各路仅 106–117 帧，覆盖窗口约 4% |
| 原短录制 WS-20261029_164400-31241c87 | 初版浏览器测试误报通过，各 D405 流实际上仅录 1 帧；已明确作废该验收并更正本轮生成的 run.json 为 incomplete，保留原元数据副本和原 MCAP |
| 修复后故障录制 WS-20261029_165249-f366f2f8 | 工业 126 帧、鱼眼 189 帧、D405 每路 146 帧；停流后自动停止，writer 和 MCAP 正常关闭，网页及 run.json 均为 incomplete |
| 故障批次回放 | 六路已保存内容均可读取；暂停、继续、停止、重播、刷新恢复通过，不改变源 MCAP |

`UVC -71` 是本机内核的实测传输错误。当前证据不能单独确定线缆、接口、供电、主控、设备或底层驱动中哪项是根因。已请求现场确认/更换 USB 3 数据线和直连端口。本轮只复位指定 D405 USB 设备，没有复位共享 USB 控制器；临时 power/control 测试无改善，结束后恢复原值 `auto`，没有安装 udev/systemd、升级固件或覆盖系统 SDK。

保存波形测厚显示的 15.1470 mm 来自已有文件，不是本轮 EMAT 实测。三相机配置使用 `/dev/emat`，缺失时实时测厚明确失败并清除旧厚度。雷达、涡流、EMAT、底盘实机与算法精度不在本轮三相机硬件通过范围内。

## 软件与历史诊断

- 离线七项检查全部通过：core 51、data 76、视频消费者 13、RRD/MCAP 24、通信桥 7、真实 ROS 快照 1，另含两种 anchor 的 C++/DDS 涡流模拟链路。
- JavaScript 单测 8 项通过。历史 MCAP 浏览器回放 2 项通过；本轮故障 MCAP 浏览器回放 2 项通过；真实缺流拒绝录制 1 项、录制中停流保护 1 项通过。
- P2E0/E1/E2/E3/E4/P2G1 六组诊断重新计算并写入本工程 runtime，基线状态均 matched，先前缺失的输出已生成。冻结输入仅从明确指定的旧数据根只读读取，未复制旧报告充当新结果。
- 冻结配置 SHA 保持原值：对照显式 `INSPECTION_BASELINE_FIXTURES` 中 SHA 匹配的旧 YAML，逐字段核验仅 `camera.sdk_root` 迁移为 null，记录新旧哈希与差异；拒绝算法参数变化。当前 tracker 使用现有数值、状态和路径基线验收，同时记录当前及历史实现 SHA。历史 timing 文件仅是历史输入，不能当成本轮速度测量。
- 修复诊断脚本跨目录 `relative_to` 异常，以及历史 timing/现场快照误指向新输出目录的问题。
- 合并回归的精确计数 DDS fixture 曾在固定 20 Hz 发送时触发既有 latest-frame 替换（10 对匹配、9 对处理、1 对丢弃）。改为双向发现完成后逐帧等待结果，保留精确十帧、顺序和零丢弃断言；生产算法队列策略未改。

完整合并回归 **370 passed、6 skipped（58.19 秒）**，见 `final_regression_verified.log`。跳过项为 SDK 直连相机专项 1 项及显式门控的实机/运行中服务测试 5 项；其中故障预检、故障录制、两项回放已独立启用验证，健康三相机验收仍受停流阻塞。硬件门控测试的跳过不能作为硬件通过证据。最终静态检查 passed，9 个安装资源清单无缺失。

## 复用与恢复

启动三相机上位机：

```bash
cd /media/mcap_data/project_east_eleair_refactor
./apps/web_console/run.sh --config config/communication/cameras.yaml
```

本轮结束时保留该配置的服务，HTTP 8080、WebSocket 9001、ROS 域 77；无录制/回放任务运行。再次启动会提示已有实例，避免重复占用设备。访问 `http://192.168.127.10:8080` 或 `http://192.168.1.148:8080`。当前 D405 失流时录制预检会拒绝通过。

现场处理连接后，在已启动服务的同一配置下验证：

```bash
source deploy/env.sh s100p
export INSPECTION_SYSTEM_CONFIG="$PWD/config/communication/cameras.yaml"
python3 tools/system/check_hardware_streams.py --seconds 180 \
  --output .runtime/s100p/validation/after_reconnect.json
SYSTEM_LIVE_TEST=1 python3 -m pytest -q \
  tests/operations/test_live_workflow.py::test_real_buttons_video_collection_reconnect_and_thickness
```

先通过六路持续性与录制帧率，再验证原 1280×720×30 配置及更长时长。当前 640×480×15 是诊断设置，不等价于恢复原性能。

复现软件测试的输入环境（只用于测试，不写入正式启动环境）：

```bash
source deploy/env.sh s100p
export FIELD_VISION_EVIDENCE_ROOT=/media/mcap_data/east_eleair_plus/tank_vision_inspection/output
export INSPECTION_BASELINE_FIXTURES=/media/mcap_data/east_eleair_plus
export PATH="$TANK_RUNTIME_ROOT/tooling/node-v22.15.0-linux-arm64/bin:$PATH"
./deploy/verify_offline.sh
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 TANK_VISION_RUN_ROS2_E2E=1 \
  python3 -m pytest --import-mode=importlib -q \
  packages/tank_vision/tests packages/d405_seam_tracking/tests tests/operations tests/data \
  tests/system/test_wrappers.py tests/system/test_portability.py \
  tests/system/test_hardware_stream_report.py tests/system/test_frozen_config_migration.py
```

六组诊断通过 `python3 -m experiments.vision.scripts.<模块>` 运行，顺序为 `diagnose_guided_pair_unary`、`diagnose_guided_dp_provenance`、`experiment_center_transition_weight`、`diagnose_reference_selection_error`、`experiment_reference_anchor_candidate`、`experiment_normalized_step_anchor`。输出目录非空时主动拒绝覆盖；当前结果已生成，重跑前须另存本次输出，不能改写冻结输入。

## 证据目录

`.runtime/s100p/validation/20261029/`：

- `streams_full.json`、`streams_low.json`、`streams_reset.json`：真实 180 秒数据流观察。
- `hardware_low.log`、`hardware_reset.log`、`guard_hardware.log`、`kernel_final.log`：驱动及内核错误。
- `short_recording_original_run.json`、`short_recording_invalidated.json`：作废的短录制验收及原元数据。
- `interruption-acceptance.json`、`browser_interruption.log`：真实录制中断保护。
- `missing-camera-acceptance.json`、`browser_fault.log`：缺流时预检拒绝。
- `browser_replay.log`、`browser_new_run_replay.log`、`guard_replay-acceptance.json`：历史及本轮 MCAP 浏览器回放。
- `offline_with_evidence/verification.json`、`javascript.log`、`final_regression_verified.log`、`reliability.log`、`static_final.json`、`metadata.log`：软件验证。
- `p2e0.log` 至 `p2e4.log`、`p2g1_fixed.log`：重新生成诊断；完整结果在 `.runtime/s100p/analysis/`。
- `supervisor.pid`、`supervisor_guard.log`、`final_status.json`：当前服务与最终任务状态。

早期失败日志和误判验收仍保留用于审计，不能只抽取其中的 passed 作为最终结论。
