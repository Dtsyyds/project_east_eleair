# East Eleair 迁移工程

现有代码已归入统一布局：apps 为网页与 CLI，packages 为不依赖 ROS 的业务/算法核心，ros2_ws/src 为 ROS 适配、驱动、接口和 bringup。原有 27 包保留，新增 inspection_operations。详细去向见 [迁移映射](docs/architecture/migration_map.md)。

```bash
cd /media/mcap_data/project_east_eleair_refactor
source deploy/env.sh s100p
./deploy/preflight.sh --json                  # 无设备检查；缺新 overlay 时返回 2
./deploy/build.sh --platform s100p --plan     # 只打印构建命令
./apps/inspection_cli/inspection --help
./deploy/run.sh --help
```

实际构建命令为 `./deploy/build.sh --platform s100p --offline`，当前尚未执行完整构建。x86/rk3588 使用相应 profile，须先准备对应架构依赖；现有 deb/wheel 主要为 ARM64。
完成下一轮依赖与验证后，`./apps/web_console/run.sh` 启动现有整套系统，普通执行会启动设备进程。当前没有硬件接入，本轮只检查其帮助入口。
CLI 的 collect/stop/status/replay/validate/handoff/history 与网页共享 inspection_runtime；用 `inspection COMMAND --help` 查看各命令。

- [配置入口/覆盖顺序](config/README.md)：communication/system.yaml、vision、d405、collection 与 ROS 包配置目录索引。
- [构建/运行/兼容与回退](deploy/README.md)：生成物统一 `.runtime/<platform>`；外部数据使用 FIELD_DATA_ROOT。
- [迁移完成报告](docs/architecture/migration_completion.md)：静态/实际执行/依赖阻塞/未执行区分。
- [下一轮验证计划](docs/validation/post_migration_validation_plan.md)：干净构建、mock、录放、网页、定位、实机。
- [依赖记录](third_party/DEPENDENCIES.md)：二进制不入 Git；来源、SHA、ELF 架构见 dependency_inventory.json。

本轮没有完整系统构建或硬件验收。docs/history 保存原报告与根说明书；docs/system、data、vision、d405、emat 保留已有设计/算法资料，其中历史命令与成功状态不代表新布局已通过验证。执行入口以本 README、deploy/README 和下一轮验证计划为准。
原工程仅用于只读核对；运行入口不依赖它的源码、build、install 或虚拟环境。保留模型未提供、粗标定证据缺失、新设备/网页点云/通用融合未实现的准确状态。
