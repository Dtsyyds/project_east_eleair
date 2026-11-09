# M0 工程迁移基线

审计日期：2026-10-22（Asia/Shanghai）。本次交付仅为审计与迁移准备，未移动源码、未复制依赖、未安装软件、未执行构建或启动设备。

## 1. 仓库身份与约束

| 项目 | 已核实结果 |
|---|---|
| 新工作目录 | `/media/mcap_data/project_east_eleair_refactor` |
| 当前分支 | `refactor/architecture` |
| HEAD | `b34a3c29c62500915c302b340cb4d51880b00980` |
| 提交说明 | `Initial import of East Eleair project source` |
| 审计开始工作区 | `git status --short --branch` 仅显示 `## refactor/architecture`，无改动 |
| 原工程（只读） | `/media/mcap_data/east_eleair_plus`，分支 `main`，HEAD 与新目录相同 |
| 仓库关系 | `git worktree list` 确认为同一仓库的两个 worktree；新目录 `.git` 是 worktree 指针文件 |
| AGENTS.md | 检查 `/AGENTS.md`、`/media/AGENTS.md`、`/media/mcap_data/AGENTS.md`，并递归查找新旧工程（不进入 `.git`），均不存在适用文件 |
| 源文件对比 | 新目录审计开始的全部 5,685 个 Git 文件在原工程均存在，逐文件 SHA-256 全部相同 |

这里的“全部相同”仅覆盖 Git 文件，不能推导出非 Git 依赖或运行环境完整。原工程始终只读；临时清单、colcon 发现日志保存在 `/tmp`，不作为长期交付依赖。

## 2. 现有布局与真实边界

当前仍是 `physic_bev/`、`tank_vision_inspection/`、`tank_inspection_system/`、`system_websocket/`、`d405_seam_tracking/` 等旧布局；目标 `apps/`、`packages/`、统一 `ros2_ws/` 尚未建立。本轮仅新增 `docs/architecture/` 下三份文档。

- 29 份受 Git 管理的 `package.xml`：27 个 ROS 包，2 个第三方 CMake 项目（`sophus`、`librealsense2`）。包名、完整声明依赖、源码与目标位置见 [迁移映射](migration_map.md)。
- 显式扫描三个现有 ROS 源码根目录的 `colcon list` 成功，返回 25 个包。`realsense2_rviz_plugin`、`realsense2_ros_mqtt_bridge` 目录带 `CATKIN_IGNORE`，不在返回结果中。保留标记，不默认启用。
- `vikit_common/package.xml` 没有 `export/build_type`；尽管声明 `ament_cmake`，当前 colcon 将其识别为 `ros.catkin`。这是实际发现，未尝试构建，迁移时须核对并修正元数据与实际 CMake 的一致性。
- `physic_bev/src/FAST-LIVO2` 的包名为 `fast_livo`；`realsense2_rgbd_plugin` 目录的包名为 `realsense2_rviz_plugin`。不得按目录猜测或更改包名。
- `tank_inspection_system` 带 `COLCON_IGNORE`，是编排、部署、验证集合，不是 ROS 包。`system_websocket` 顶层是应用；真正的 ROS 通信包位于 `physic_bev/src/system_websocket/{agv_bridge,agv_protocol}`。
- 非 ROS Python 分发为 `tank-vision-inspection`（导入 `tank_vision`）和 `d405-seam-tracking`（导入 `d405_seam_tracking`）。二者不能与 ROS 包目录混为一谈。

## 3. 当前机器与依赖状态

机器架构 `aarch64`，Python `3.10.12`；`/usr/bin/colcon` 与 `/opt/ros/humble/bin/ros2` 存在，`/opt/ros/humble`、`/opt/tros/humble` 存在。继承环境 `ROS_DISTRO=humble`，`AMENT_PREFIX_PATH=/opt/tros/humble:/opt/ros/humble`，无激活 venv，无旧工程 overlay 出现在本次检查的 AMENT/CMAKE/PYTHON/LD 搜索路径中。这只是命令与路径存在性检查，不是 ROS、TROS、ABI 或功能验收。

主要缺失与风险：

| 编号 | 已核实发现 | 后续影响 |
|---|---|---|
| B01 | 原工程离线清单 33 个归档（32 deb + 1 wheel）均实物存在并与清单 SHA-256 匹配；新目录 0/33 | 当前 `build.sh` 在读取第一个缺失归档时即会失败（依据代码，未执行） |
| B02 | 原工程 D405 offline 中另有 39 个 wheel；新目录缺失 | 不能用原 `.venv` 代替重建；其中旧业务 wheel 还须从迁移后的权威源码重做 |
| B03 | 两套 MindVision SDK 共 12 个 `libMVSDK.so` 架构变体均只存在于原工程 | 新目录有绑定/头文件并不等于 SDK 完整；须显式指定匹配的只读 SDK 根目录 |
| B04 | `/usr/lib/libMVSDK.so` 哈希与旧 SDK arm64 一致，与 2.1.0.50 arm64 不同 | 系统 SONAME 搜索不能证明选中新版 SDK |
| B05 | `system_websocket/run.sh` 无条件 source 缺失的 `physic_bev/install_lifecycle/setup.bash` | 新 worktree 当前启动链不完整，未启动确认 |
| B06 | `tank_vision_core` 跨目录安装算法；`tank_vision_ros/setup.py` 跨目录收集 4 个配置资源 | 拆分目录必须同时修改安装路径，详见审计 P05/P06 |
| B07 | FAST-LIVO2 直接链接 `../../install/vikit_*/lib/*.so`；编译嵌入源码根目录 | 统一工作区与安装后运行均存在路径耦合 |
| B08 | 多份配置/脚本保留 `/home/sunrise/east_eleair`；多个脚本推导旧兄弟目录、注入源码 PYTHONPATH | 移动上层目录会改变配置、资源、导入和溯源结果 |
| B09 | 未发现可用模型权重文件；平台配置 `model_path: null` | 经典算法与模型后端验收必须分开；模型后端缺少权重和运行验证 |
| B10 | `rough_extrinsics.yaml` 明示 `calibration_stage: rough`、`production_valid: false` | 文件存在和哈希一致不代表生产标定通过 |
| B11 | 粗外参引用的 `正视图.png`、`侧视.png`、`俯视.png` 在新旧工程均未找到 | 无法实物复核配置记录的证据图片 SHA；需后续补齐来源 |

完整归档、SDK、标定数据核对及路径证据见 [路径与依赖审计](path_and_dependency_audit.md)。

## 4. 已执行的验证与边界

| 检查 | 实际结果 | 不能据此认定 |
|---|---|---|
| 分支、HEAD、worktree、初始 status | 已核实，见上表 | 已提交本轮文档 |
| 原/新 Git 文件 SHA-256 | 5,685/5,685 相同 | 非 Git 文件完整 |
| XML 解析 | 29 份 package.xml 均可解析 | 声明依赖完整或依赖已安装 |
| colcon 发现 | 25 个 ROS 包；退出码 0 | 27 个包均能构建 |
| Python AST / `bash -n` | 选定范围 131 个文件全部通过，未生成 pycache | Python 导入、launch 求值或设备运行通过 |
| 离线清单哈希 | 原工程 33/33 匹配；新工程全部缺失 | 已安装这些依赖或 ABI 相容 |
| 关键标定/相机参数、SDK 哈希 | 见依赖表；只读文件核对 | 标定有效、相机被成功打开 |
| 实际构建、colcon test、pytest、ROS 节点、采集回放、WebSocket、设备 | 本轮均未执行 | 历史报告中的 pass 可作为本轮结果 |

语法检查范围：非厂商源码中的 `tank_inspection_system/scripts/`、`physic_bev/scripts/`、`system_websocket/`、视觉与 D405 ROS 源码、各 `setup.py`、`launch/*.py`。CMake/setup/launch 的静态检查清单见审计附录。CMake 未进行 configure/build。

原工程旧 `build*`、`install*`、`log*`、`.runtime`、venv 只用于确认存在，不作为迁移输入；没有复制或 source。新目录未发现实际 build/install/log/venv 运行树。初始 Git 快照已有 MindVision demo 的 `config.status`、`config.log`、`autom4te.cache`、`.deps` 等历史生成文件，这些属于已跟踪遗留物，不能当作本轮构建结果或可共享依赖。

## 5. M1 进入条件与执行次序

建议先迁移 `inspection_interfaces`、`eddy_driver`、`inspection_livox_adapter`、`inspection_sync`、`inspection_mock_sensors`，总计 36 个受 Git 管理文件；无需先迁移厂商 SDK 或视觉算法。完整逐文件清单、同步引用、干净构建和无硬件验证步骤见 [M1 第一批方案](migration_map.md#m1-第一批迁移方案)。

M1 应以新输出目录构建这 5 包，保留原包名、消息定义、话题、可执行程序名；先通过消息导入和基础 C++ 单测，再跑合成 ROS 数据链路。后续批次再处理算法/配置安装、设备驱动、应用和部署入口。未解决本批构建与引用错误前，不将整机功能标为通过。
