# 全量迁移完成记录

日期：2026-10-22。工作树 `/media/mcap_data/project_east_eleair_refactor`，分支 `refactor/architecture`。
初始实际 HEAD 为 `b34a3c29c62500915c302b340cb4d51880b00980`。M0 的审计/首批限制已被全量迁移授权取代。
本轮完成代码、配置、工具、文档归位以及迁移引用修复；完整构建、系统录放和硬件验收仍按用户约定留到下一轮。

## 文件与包完整性

- 原 5,685 个 Git 文件全部有唯一目标，目标均存在且进入 Git。机器映射 `migration_files.json` 保存原/目标 SHA、模式、抽取和兼容关系；没有无说明删除。
- 5,474 个目标内容与基线相同；211 个基线文件有路径、入口、安装、语法或职责调整。Git 对相同内容副本的 rename 配对可能与来源账本不同，以逐文件映射和 SHA 为准。
- M0 之外补充恢复 5 个原工程 `tank_vision/outputs/*.py`。原 `tank_vision_inspection/.gitignore:9 output*/` 误忽略了真实源码；已改为根级输出规则。来源及原/目标 SHA 在 `migration_supplemental_files.json`，overlay.py 仅去掉末尾多余空行，其余恢复文件逐字相同。
- 原 27 个 ROS 包全部保留真实包名、既有消息定义与依赖；新增 inspection_operations，实际 `colcon list --base-paths ros2_ws/src` 发现 28 包。完整逐包去向见 migration_map.md 和 ros_packages.json，全部依赖见静态结果 JSON。
- 原 msg/srv/action 文件内容 SHA 不变。RealSense 整体 5 包与 rpg_vikit 整体 3 包保留；可选 RealSense CATKIN_IGNORE 改名保存，profile 控制选择，不靠隐藏包制造成功。
- 两个非 ROS 第三方 package.xml 项目仍在 third_party，不混入 ROS 扫描；分组目录未添加 package.xml。

## 已完成职责和入口

apps/web_console 保留正式网页协议/静态资源；apps/inspection_cli 调用同一 inspection_runtime，包含启动、采集、停止、状态、回放、校验、交付、历史。
原 Operations/ReplayController/进程编排与 physic 业务进入 runtime；EMAT 串口协议、波形解析和计算进入 emat_core；消息发布、ROS 监控/录像/bag 读取进入 inspection_operations。
公共 tank_vision 与 d405_seam_tracking 导入名保留；D405 ROS 实现进入 d405_seam_ros，旧公共模块薄转发。

六个 Python 分发有真实 setuptools 元数据和入口。tank_vision 只有 packages/tank_vision 一处权威源码，pip 可编辑安装；tank_vision_core 不再跨目录安装第二份源码，而检查实际导入来源。
部署入口从自身位置定位 checkout；build/env/run/preflight 及 CLI 支持无设备副作用的 --help。CLI 通过 exec 保持退出码、参数和终止信号。
build 明确扫描 ros2_ws/src，按 config/platforms 选择依赖闭包；新 build/install/log/native/venv/sysroot/ROS logs 进入本工作树 `.runtime/<platform>`，FAST-LIVO 输出也不再落入源码 Log。

配置入口与覆盖顺序见 config/README.md。中央视觉配置由 ROS setup 安装到 share，原包内配置/launch/resource 仍由所属包安装；不存在两套人工同步的权威配置。
MindVision 正式适配显式定位 SDK 版本、参数文件和 runtime 数据目录；加载相同序列号的原 mvdat 后再应用原显式节点参数。依据是保留的厂商头文件与 ELF 符号，设备端行为未验证。

兼容入口见 compatibility_manifest.json：deploy/compat 的旧命令转发、tools/data 的开发命令转发、D405 ros_node 与 inspection_tools.video_index 的公共兼容导入。旧顶层业务目录不保留第二套实现。旧 --wireless-only 转译为新依赖选择；原 verify_offline 工作流迁到 deploy/verify_offline.sh，结果写新验证目录。
原根 README 和过时审计工具原文归档，旧报告没有被新测试改写成成功。

## 依赖准备与缺口

311 个离线/SDK 分发文件，723,804,201 字节，已从原工程只读复制并校验目标 SHA；33 个系统归档符合原固定清单，52 项额外匹配 M0 表内 SHA。两套 SDK 分版本保存，ELF 架构和软链接记录在 third_party/dependency_inventory.json；全部分发文件被 Git 忽略。
没有搬入原 build、install、log、虚拟环境、解包 sysroot 或第三方旧编译库。没有修改系统 ROS、驱动、udev、服务。原工程及 main 未修改，未推送或合并。

仍需明确准备：

- Rerun 0.36.2 及完整独立 wheel 集/新 rerun_venv；NumPy2 与 D405 的 1.26.4 隔离。当前 Rerun 环境缺失，不能宣称 RRD 转换可运行。
- 新 ROS 安装产物与 generated messages；当前 eddy_driver/inspection_operations 导入受阻属于未构建，而不是源包丢失。
- Node.js：本机没有 node；5 个 JS 文件的语法检查及 node:test 未执行。网页正式 JS 本体逐字保留，测试 require 路径已修正。
- 系统 PyYAML 5.4.1 不满足声明；仅新 metadata_venv 从已核验本地 wheel 安装既有固定 6.0.3，未改系统。正式 build 有相同固定版本安装步骤。
- ONNX/RKNN/BPU/训练模型权重仍未提供，默认 null；三张 rough 外参证据图片 `正视图.png`、`侧视.png`、`俯视.png` 新旧工程均缺，原哈希未改，production_valid=false。
- 历史大型视觉 output 与采集数据未复制，通过 FIELD_VISION_EVIDENCE_ROOT / FIELD_DATA_ROOT 显式接入；历史源码复现实验需要 INSPECTION_BASELINE_FIXTURES，不回退正式运行源码。
- x86 对应版本归档、native 依赖闭包/ABI、平台 SDK 与设备实际参数须下一轮核验。保留上游脚本的既有静态警告见 upstream_static_findings.json；未借迁移重写第三方功能。

## 实际验证结果

| 状态 | 本轮实际执行及结论 |
|---|---|
| 静态检查通过（可执行范围） | 527 个 Python AST、58 个 Bash、46 个 YAML、6 个 TOML、28 个 ROS package.xml；JSON 文件检查明细见 migration_static_results.json；无本项目语法错误 |
| 静态完整性通过 | 5,685 基线目标 + 5 补充源码均在；原消息 SHA/包名/依赖保留；无重复 ROS 包；9 份 ROS Python setup 的声明资源均存在 |
| 实际执行通过 | 六分发可编辑安装到新 metadata_venv；20 项帮助/平台 build plan/无 ROS 核心导入/视觉参考加载检查通过；核心导入禁止 ROS 且来源全部为新位置 |
| 实际执行通过 | 针对性现有测试累计 81 passed；参数、退出码和 SIGTERM 转发、业务/交付、配置、写盘与来源检查有实际结果；SDK 路径仅用 fake API 验证，无设备访问 |
| 依赖阻塞/跳过 | 2 项 ROS replay 测试首次运行缺新 generated message/适配安装；其余纯逻辑选择后通过，未把这两项计入通过。1 项相机测试按原条件跳过。JS 因无 Node 未执行 |
| 预检准确报告缺失 | deploy/preflight.sh 返回 2，报告新 overlay、Rerun 与当前默认 Python 版本依赖问题；该返回码是预期缺失报告，不代表就绪 |
| 未执行 | 全量 CMake/colcon 构建、完整测试套件、DDS mock 集成、MCAP 录放/RRD、浏览器交互与定位回归 |
| 硬件未验证 | 相机/LiDAR/D405/涡流/EMAT/底盘、SDK ABI、实际标定精度及整机运行 |

证据：docs/validation 下 migration_static_results.json、migration_smoke_results.json、migration_test_results.json、installation_metadata.json、colcon_discovery.txt、path_reference_scan.json。
首次干净环境安装曾暴露 setuptools 因 PYTHONPATH 已有源码而不写 editable .pth 的问题；已修复构建/安装步骤并重新在去掉 PYTHONPATH 的环境安装和检查，最终 20 项检查均通过。
路径扫描保留的旧工程字符串仅为拒绝旧 overlay 的 guard、负向测试、显式历史 fixture；历史文档/厂商示例与外部数据路径另行分类，不作为当前运行 fallback。
`git diff --check` 通过。二进制忽略规则及所有基线目标进入 Git 已复核。

## 本地提交与下一步

- `9495fce27b0ce6b33a19db323ebf1c9a67439cb5`：保存 M0 三份文档、起始状态和初始文件账本。
- `712e871`：全仓代码/配置/工具/文档迁移、业务抽取、入口/依赖/路径修复及实际检查记录。
- 本报告及最终进度记录由后续 `docs: finalize migration completion and validation handoff` 本地提交保存；准确提交号可执行 `git log --oneline b34a3c2..HEAD`，最终答复同时提供。

实现提交后的工作区曾确认干净；交付文档提交后再复核工作区状态。main 仍为 b34a3c29c62500915c302b340cb4d51880b00980。未改写历史、未推送。
下一轮按照 docs/validation/post_migration_validation_plan.md 执行干净构建、安装资源核验、纯软件/mock、录放/网页/定位，最后独立进行硬件接入验收。新设备、网页点云传输和通用多传感器融合没有在本轮新增或伪造实现。
