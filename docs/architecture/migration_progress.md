# 全量迁移进度

## 2026-10-29 三相机与采集可靠性续作

新增独立三相机配置与批次配置追溯；修复存储异常导致状态/自动停止失败，以及停流后仅凭非零帧数误判完整的问题。真实 D405 录制中断测试已验证自动关闭 MCAP、批次标记 incomplete，故障批次六路浏览器回放通过。历史 P2E0/E1/E2/E3/E4/P2G1 诊断已重新生成并匹配基线，原冻结 SHA 未修改，仅允许可审计的 SDK 路径迁移。合并回归 **370 passed、6 skipped**，离线七项全部通过，JS 8 项通过，静态和安装资源检查通过。

**整机硬件验收仍未完成**：工业约 10 Hz、鱼眼约 15 Hz 持续正常；D405 在 1280×720×30 和 640×480×15 均出现 UVC `-71` 后停流，单设备复位/临时关闭省电未解决。待现场处理 USB 3 连接后复测。当前保留三相机配置的上位机服务，缺 D405 时严格拒绝录制；默认 system.yaml 整机要求保持。

接续以 [本轮报告](../validation/20261029_camera_refactor.md)、`.runtime/s100p/validation/20261029/final_status.json`、`supervisor.pid` 和 `git status` 为准。下文是此前状态，其中“12 项历史诊断未闭环”已由本轮重生成与回归闭环；D405 物理链路阻塞仍存在。

## 2026-10-28 实机与功能复现续作

新工程 S100P overlay、独立 SDK/Rerun 环境已建立；完整离线七项验证、历史 MCAP 浏览器回放、90 条新旧算法输出对比通过，短时相机网页录制/重连/保存波形测厚通过。已修复 D405 SDK 轮询与跨流时间戳误判、DDS 运行配置和若干迁移路径问题。

**整体验收未完成**：工业相机/鱼眼持续正常；D405 出图后 USB `-71` 停流；涡流零帧；EMAT 串口 EIO/握手响应错误。已请求现场检查接线与 Hub 供电，待回复。12 个历史专项诊断产物/冻结 SHA 测试仍未闭环，不可将离线脚本通过解释为全部历史功能已验收。正式服务已恢复无雷达但仍要求涡流的配置，实际整机预检失败，未放宽判定。

详情与接续步骤见 [实机和复现报告](../validation/20261028_hardware_and_parity.md)。恢复工作先查该报告、`.runtime/s100p/validation/20261028/resume/supervisor.pid` 和 `git status`。以下为原迁移阶段记录，其中“未构建/未验证”只描述当时状态。

## 原迁移阶段记录

分支 refactor/architecture；初始 HEAD b34a3c29c62500915c302b340cb4d51880b00980。
M0 检查点 9495fce；全量实现检查点 712e871；最终报告提交见 HEAD 的 `docs: finalize migration completion and validation handoff`。
未发现适用 AGENTS.md。原工程仅只读，main 保持 b34a3c2；不推送、不部署、不改系统配置。

本轮代码迁移完成：5,685 原 Git 文件逐项归位；另恢复 5 个被原忽略规则遗漏的 outputs 源文件；原 27 ROS 包保留并新增 inspection_operations；网页/CLI/runtime/EMAT/算法/配置/工具/文档全量归位。没有剩余源目录移动批次。
新构建、环境、运行、预检、回退及兼容入口已建立；311 个离线/SDK 分发校验完成，未复制旧构建产物或虚拟环境。
527 Python / 58 Bash / 配置与元数据静态检查通过；28 包实际发现；9 份 ROS Python 安装资源存在；20 项安全 smoke 通过；81 项针对性测试通过，1 项相机跳过，2 项 ROS 测试因未构建受阻。

阻塞/下一轮事项：Node.js、Rerun 独立依赖环境、新 ROS overlay/generated messages、模型与 3 张粗标定证据、历史数据/源码 fixture 显式接入。完整构建、系统 mock/录放、网页、定位、硬件验证本轮未执行。
准确证据和提交记录见 migration_completion.md；后续命令与判定标准见 docs/validation/post_migration_validation_plan.md。
恢复后先读本文件、completion 和 git status；不要重跑 /tmp 迁移脚本或重复移动源码。只按下一轮明确的验证范围继续。
