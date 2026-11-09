# 2026-10-28 非雷达设备与重构功能验证

截至 18:03（Asia/Shanghai），**尚未通过“除雷达外所有设备在线可用、原工程所有功能完整复现”的整体验收**。已完成软件修复、离线回归及短时实机录制；仍有 D405 USB 停流、涡流无数据、EMAT 串口通信失败，以及历史实验复现尚未闭环。

工程：`/media/mcap_data/project_east_eleair_refactor`，分支 `refactor/architecture`，基线 HEAD `e2ff531`，本轮修改尚未提交。平台 S100P / ARM64 / ROS Humble。旧工程只读，用作明确指定的测试输入；当前服务、ROS overlay、SDK 构建均来自新工程。

## 实机结果

| 设备 | 实际观测 | 验收结论 |
| --- | --- | --- |
| MindVision MV-SUS1207GC，046041920527 | 1920×1200，约 10 Hz；75 秒观察收到 666 帧，采样时间戳无倒退，驱动未报告采集错误 | 本次连续出图通过 |
| icSpring 鱼眼，2993:0858 | 640×480，约 15 Hz；75 秒观察收到 1082 帧 | 本次连续出图通过 |
| RealSense D405，SDK 序列号 260322276866 | 修复后彩色/深度/左右红外均约 15 Hz，分别收到 931/942/941/941 帧；约第 66 秒全部停止，内核 UVC `-71` | 短时出图及录制通过，稳定性失败 |
| 涡流 | 未发现原设备 1a86:55d3（5B91074919），`/dev/eddy` 不存在，`/eddy/data` 零帧 | 未通过；需确认实际连接设备与串口角色 |
| EMAT 电磁超声 | 三个 CH340 在 USB 控制请求阶段出现 `-110`，串口打开报 EIO；控制器重枚举后曾短暂可读，配置端口发送 `8101` 收到 `fe`，协议要求 `01` | 未通过；不能以串口枚举或有字节视为设备可用 |
| 雷达及其 IMU | 按用户现场条件禁用启动 | 本轮不作实机验收 |

USB 控制器重新绑定曾恢复相机/串口，但故障复现；已请求现场检查 D405 USB 3 线缆/接口和串口 Hub 供电、重新插接，并确认 Hub 1/2/3 口对应设备，尚未收到回复。USB 错误证明传输链路失败，尚不能仅凭错误码断定线缆、供电、主控或设备中哪一项是根因。

D405 当前配置 640×480×15 是排障配置，**不等价于原工程 1280×720×30 性能恢复**。原分辨率实测出现 `-71` 后无帧。当前低分辨率也未通过持续性测试，不能称为已解决。EMAT 当前物理路径是待确认的候选口，`calibrated: false` 保留。

## 已修复并验证的内容

- 修复 librealsense 在关闭 global timestamp 后仍轮询固件时钟的问题；多个传感器共享时钟时，仅有启用请求才轮询。SDK 编译安装在本工程 `.runtime/s100p/native/install`，未覆盖系统 SDK；实际 ROS 进程加载路径已核对。SDK 时钟测试 6 项、83 个断言通过。
- ROS RealSense 驱动在注册通知回调后重新应用显式 `error_polling_enabled` 配置，避免 SDK 隐式重新开启 USB 错误查询。
- 修复 D405 将不同流到达顺序误判为设备时间倒退的问题：共用设备到主机时钟锚点，各流独立判断前后帧。新增交错帧和设备重置回归，两个 CTest 目标通过；修复后四路帧率一致，实机日志未再出现误判丢帧。
- 恢复 supervisor 的 Fast DDS 配置与 RMW 环境，修复新工程运行时缺少原共享内存配置的问题；保留独立系统配置入口、无雷达启动与缺失数据拒绝录制逻辑。
- 新构建明确使用 Release 和本工程原生 librealsense CMake 路径，避免缓存选中系统 SDK；修复安装包路径、历史图像输入路径、实验模块导入和 Rerun 隔离环境的测试插件加载问题。
- 新增 `tools/system/check_hardware_streams.py`，订阅真实 DDS 数据，记录帧数、帧率、最大间隔、图像内容和采样时间戳。域默认取系统 YAML，避免继承 shell 的域 23 导致误判。它不代表 EMAT 协议或算法精度验收。

## 功能复现证据

| 功能 | 本轮结果 | 边界 |
| --- | --- | --- |
| 完整离线验证脚本 | 七项检查全部通过：core 51、data 76、视频消费者 13、RRD/MCAP 24、无线桥 7、快照 1；涡流两种 anchor 的安装后 C++/DDS 链路通过 | 模拟输入验证不代表真实涡流硬件通过 |
| operations/data/wrapper | 63 项通过 | 含错误状态、录放控制与路径契约 |
| 实际 Firefox 历史回放 | 2 项通过，使用真实历史 MCAP `WS-20261019_103833-564c976e` | 覆盖视频、深度、红外、点云、IMU、涡流回放，暂停/继续/刷新；无现场雷达精度结论 |
| 实际 Firefox bridge | 3 项通过；JS 单测 8 项通过 | 控制协议使用模拟对象，无真实运动指令验收 |
| 新旧算法业务输出 | 冻结图像/空白输入的 90 条结果一致，排除耗时字段 | 验证迁移行为一致，不证明识别精度 |
| 实际相机录制与网页按钮 | 1 项通过：预览、开始/结束、重连保留录制、保存波形测厚、MCAP 完整结束 | 临时相机专用配置，不包含真实涡流；现场 EMAT 失败被正确显示 |
| D405 分析与标定 | 前次实机 72 对匹配输入处理完成，无处理异常，状态均为 LOST；本轮安装/SDK相关回归已补测 | 场景未作为有效焊缝精度样本；模型、外参、测厚标定仍不能宣称生产验收 |
| 迁移静态检查 | passed，无依赖阻塞；9 份安装资源齐全 | 静态检查不代表全功能实机验证 |

实机录制批次：`/media/mcap_data/runs/WS-20261028_175914-6ca31559`。记录工业相机 118 帧、鱼眼 178 帧、D405 各路 181 帧及 1 条**保存波形**测厚结果；网页显示 15.1470 mm 来自文件，**不是本次 EMAT 实测**。`/inspection/ultrasound/raw` 为零；实时测厚明确返回串口 EIO。

广泛算法测试首次结果为 266 passed、22 failed、6 skipped。针对路径/导入/环境/测试替身等修复，58 项复测通过，另一个宽度参考构建测试和 SDK 投影测试分别通过。仍有 12 个历史专项诊断产物/冻结 SHA 断言未闭环，涉及 P2E0/E1/E2/E3/E4/P2G1。它们需要迁移后的可审计重新生成流程；部分旧配置 SHA 对应迁移前硬编码 SDK 路径。没有更改冻结 SHA、复制旧报告充当新结果或删除失败测试。因此不能报告“全部测试通过”或“所有历史实验完整复现”。

## 当前服务与下一步

最后已退出临时相机专用验收配置，重新启动正式 `config/communication/system.yaml`：雷达关闭，涡流仍启用且为必需项；不放宽整机验收条件。supervisor 启动 PID 114989，实际 PID 记录在证据目录 `supervisor.pid`。控制台 `http://192.168.127.10:8080`（另一网卡 `http://192.168.1.148:8080`），WebSocket 9001，ROS 域 77。

正式网页预检实测拒绝通过：缺少 D405 彩色、D405 深度和涡流，工业/鱼眼与存储通过。当前没有验收录制任务运行。现场修复连接后，优先重新确认串口角色、EMAT 协议与涡流有效帧，再做原分辨率长时稳定性和完整整机采集/回放。之后补完历史诊断实验与实际焊缝/测厚标定验收。

复测连续流（同一个新工程 shell）：

```bash
source deploy/env.sh s100p
python3 tools/system/check_hardware_streams.py --seconds 180 \
  --output .runtime/s100p/validation/20261028/after_reconnect.json
```

## 原始证据

根目录：`.runtime/s100p/validation/20261028/resume/`（运行产物不入 Git）。关键文件：

- `streams_clock_fix.json`、`hardware_clock_fix.log`、`kernel_final.log`：修复后实机连续性失败与内核错误。
- `serial_after_reset.json`、`emat_start.log`：串口被动读取及 EMAT 握手失败。
- `acceptance.json`、`live_browser_clock_fix.log`：短时相机网页/录制验收和真实 EMAT 失败。
- `full_hardware_preflight.json`：恢复正式配置后实际预检失败。
- `offline_final/verification.json`：七项离线检查通过；`browser_replay.log`、`browser_bridge.log`、`javascript.log`。
- `business_comparison.json`、`algorithm_fixes.log`、`width_reference_fix.log`、`projection_sdk.log`、`algorithms_all.log`：行为对比、修复复测和未闭环诊断测试。
- `sdk_test_final.log`、`clock_fix_build.log`、`clock_fix_colcon/`、`driver_clock_tests.log`：SDK 与 ROS 编译/测试；最终时钟 CTest 另存 `driver_clock_tests_final.log`。
- `streams_full_resolution_domain77.json`：原分辨率正确域实测。较早的 `streams_full_resolution.json` 使用了错误域 23，不用作设备结论。

本轮只临时重置指定 USB 控制器和调整运行期 USB 省电选项；未升级固件、安装 udev 或 systemd 服务，未改写旧工程/旧历史数据。
