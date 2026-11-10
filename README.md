# East Eleair 迁移工程

## 底盘与循迹整合入口（2026-11-05）

现有上位机新增“底盘与循迹”页：手动调车速、WASD/ROS 键位、探头 1/2 升降与占空比、循迹启停、误差/请求速度/实际轮速/权限与停止原因。**保持原来的网页 8080 / WebSocket 通信 9001**，只在原通信入口加入底盘和循迹功能。打开主页就自动连接相机和底盘，不需要切到底盘页或分别点连接；重连不恢复运动授权。

RDK 已部署完整相机后台且最新底盘工程已构建后，一条命令启动：

```bash
bash /media/mcap_data/project_east_eleair_refactor/system_websocket/run_robot.sh --real --host 192.168.1.148
```

电脑浏览器打开终端打印的 `http://192.168.1.148:8080/`，进入“底盘与循迹”；页面自动使用原通信端口 9001。`--host` 换成电脑可达的实际 RDK 地址。默认底盘工作区是 `/home/sunrise/Desktop/Robot_Chassis_Code/robot_chassis_ros2ctrl`，通过 `--chassis-workspace` 或 `CHASSIS_WS` 覆盖。**启动不会自动启用车辆运动**：先在原“实时巡检”页面启动视觉识别，确认反馈，再在新页人工点击“启动循迹”。不加 `--real` 为预览，实际底盘不执行循迹速度。

默认两驱、域 77、硬件反馈超时 500ms、键盘独立节点关闭、MCAP 分模块录制。无需再单独运行 `ros2 topic echo` 或调用启用服务。终端 Ctrl+C 停止本次进程并收尾录制。先用 `--plan` 查看计划、`--check` 仅检查依赖；`--help` 查看参数。

传感器默认使用 `config/communication/robot_all.yaml`：启动工业相机、鱼眼、D405、MID360（含 IMU）和涡流，保留 D405 候选循迹算法。`bash system/run_robot.sh --real` 兼容入口也转发到同一启动流程。`--real` 仅决定循迹是否输出实际速度；传感器选择由 `--camera-mode all|d405|cameras` 控制，默认 `all`。临时只接 D405 时可使用 `--camera-mode d405`，三相机调试可使用 `--camera-mode cameras`。

定位与建图默认随 `bash system/run_robot.sh`（或 `system_websocket/run_robot.sh`）启动，打开主页后进入“定位与建图”即可使用。已移除 `--slam` 开关及单独打开入口；SLAM 服务使用 8082，算法仍需在页面点击“启动”。`--plan` 包含 SLAM 服务，`--check` 同时检查其依赖和端口，配置可通过 `--slam-config` 指定。

网页实时回传三路相机预览、涡流数据和雷达数据到达状态；现有网页不提供原始点云或 IMU 波形显示。点击“启动采集”将录制完整传感器话题及循迹输出，启动前会检查所需传感器数据；底盘 MCAP 仍由底盘模块独立录制。MID360 使用当前安装包的网络配置，涡流默认使用 `/dev/eddy`；未接线或缺少设备别名时页面显示无数据。EMAT 继续按需测厚，需在 `robot_all.yaml` 中配置已确认的实际串口。

本地这份副本的 `packages/inspection_runtime/src/inspection_runtime/supervisor.py` 等相机后台为零字节占位文件，不能据此宣称整套相机已可启动。新入口会明确拒绝这种不完整部署；先补齐 RDK 上已能运行的相机后台。`--no-camera` 可仅启动底盘与静态网页，相机/视觉需由其他完整部署提供。详细更新、测试与切换步骤见 [部署说明](deploy/README.md)。

现有代码已归入统一布局：apps 为网页与 CLI，packages 为不依赖 ROS 的业务/算法核心，ros2_ws/src 为 ROS 适配、驱动、接口和 bringup。原有 27 包保留，新增 inspection_operations。详细去向见 [迁移映射](docs/architecture/migration_map.md)。

```bash
cd /media/mcap_data/project_east_eleair_refactor
source deploy/env.sh s100p
./deploy/preflight.sh --json                  # 无设备检查；缺新 overlay 时返回 2
./deploy/build.sh --platform s100p --plan     # 只打印构建命令
./apps/inspection_cli/inspection --help
./deploy/run.sh --help
```

实际构建命令为 `./deploy/build.sh --platform s100p --offline`，S100P 独立 overlay 与 SDK 已建立并运行验证。x86/rk3588 使用相应 profile，须先准备对应架构依赖；现有 deb/wheel 主要为 ARM64。
日常启动沿用旧工程命令，自动加载环境和当前三相机配置，无需额外参数：

```bash
bash system_websocket/run.sh
```

网页地址由启动终端打印（默认 HTTP 8080 / WebSocket 9001），Ctrl+C 关闭整套系统。该入口默认使用 `config/communication/cameras.yaml`，要求工业相机、鱼眼、D405 彩色/深度/双红外六路数据；不接入涡流或雷达。底层 `apps/web_console/run.sh` 仍保留默认 `system.yaml` 的整机启动能力。D405 当前存在 USB 停流，三相机持续采集尚未验收；详见 [本轮实测与修复](docs/validation/20261029_camera_refactor.md)。
**只采 D405 做算法实验/训练数据时**，使用独立入口 `bash system_websocket/run_d405.sh`；网页方案应为“D405 算法实验采集”。它只启动 D405，只录制 D405 四路原始图像、标定/内部外参和巡迹输出，不要求其他设备在线。原三相机入口保留，点击视觉巡迹按钮本身不会切换录制方案。参见 [D405 采集操作说明](docs/system/d405_experiment_collection_guide.md)。

CLI 的 collect/stop/status/replay/validate/handoff/history 与网页共享 inspection_runtime；用 `inspection COMMAND --help` 查看各命令。

工业相机画面旁提供“相机调参”：读取实际参数 → 修改 → 应用预览 → 保存并沿用。保存后按相机序列号恢复曝光、增益和色彩设置，上位机重启与后续视觉任务共用；采集和回放期间禁止修改。参见 [操作说明](apps/web_console/USAGE.md) 和 [调参实测](docs/validation/20261029_camera_tuning.md)。

上位机“实时巡检 → 巡迹结果”提供启动视觉巡迹、停止视觉巡迹、保存实验样本。使用既有 D405 Color + Depth，被动运行且不控制机器人；原始样本与离线索引保存到数据盘 `vision_experiments/`，连续录制仍使用原采集流程。详见 [本轮接入验证](docs/validation/20261030_vision_controls.md)。

- [D405 焊缝实验采集操作说明](docs/system/d405_experiment_collection_guide.md)：现场逐步操作、样本核对、异常处理和数据交接。
- [配置入口/覆盖顺序](config/README.md)：communication/system.yaml、vision、d405、collection 与 ROS 包配置目录索引。
- [构建/运行/兼容与回退](deploy/README.md)：生成物统一 `.runtime/<platform>`；外部数据使用 FIELD_DATA_ROOT。
- [迁移完成报告](docs/architecture/migration_completion.md)：静态/实际执行/依赖阻塞/未执行区分。
- [下一轮验证计划](docs/validation/post_migration_validation_plan.md)：干净构建、mock、录放、网页、定位、实机。
- [依赖记录](third_party/DEPENDENCIES.md)：二进制不入 Git；来源、SHA、ELF 架构见 dependency_inventory.json。

docs/history 保存原报告与根说明书；docs/system、data、vision、d405、emat 保留已有设计/算法资料，其中历史命令与成功状态不代表当前硬件已通过验证。当前证据以带日期的 validation 报告为准；执行入口以本 README 和 deploy/README 为准。
原工程仅用于只读核对；运行入口不依赖它的源码、build、install 或虚拟环境。保留模型未提供、粗标定证据缺失、新设备/网页点云/通用融合未实现的准确状态。
