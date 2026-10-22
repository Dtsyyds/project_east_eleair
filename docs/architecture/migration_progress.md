# 全量迁移进度

分支 refactor/architecture；初始 HEAD b34a3c29c62500915c302b340cb4d51880b00980。
M0 检查点 9495fce；全量实现检查点 712e871；最终报告提交见 HEAD 的 `docs: finalize migration completion and validation handoff`。
未发现适用 AGENTS.md。原工程仅只读，main 保持 b34a3c2；不推送、不部署、不改系统配置。

本轮代码迁移完成：5,685 原 Git 文件逐项归位；另恢复 5 个被原忽略规则遗漏的 outputs 源文件；原 27 ROS 包保留并新增 inspection_operations；网页/CLI/runtime/EMAT/算法/配置/工具/文档全量归位。没有剩余源目录移动批次。
新构建、环境、运行、预检、回退及兼容入口已建立；311 个离线/SDK 分发校验完成，未复制旧构建产物或虚拟环境。
527 Python / 58 Bash / 配置与元数据静态检查通过；28 包实际发现；9 份 ROS Python 安装资源存在；20 项安全 smoke 通过；81 项针对性测试通过，1 项相机跳过，2 项 ROS 测试因未构建受阻。

阻塞/下一轮事项：Node.js、Rerun 独立依赖环境、新 ROS overlay/generated messages、模型与 3 张粗标定证据、历史数据/源码 fixture 显式接入。完整构建、系统 mock/录放、网页、定位、硬件验证本轮未执行。
准确证据和提交记录见 migration_completion.md；后续命令与判定标准见 docs/validation/post_migration_validation_plan.md。
恢复后先读本文件、completion 和 git status；不要重跑 /tmp 迁移脚本或重复移动源码。只按下一轮明确的验证范围继续。
