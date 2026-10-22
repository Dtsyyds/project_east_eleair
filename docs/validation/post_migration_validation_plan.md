# 下一轮统一验证计划

这里的命令均为**拟执行**，本轮没有执行完整构建、DDS mock 链路、录放、浏览器交互、定位或硬件验收。本轮实际结果另见 migration_static_results.json、migration_smoke_results.json 与 migration_test_results.json。
在独立工作树或全新的平台 runtime 中执行，不复用原工程 build/install/venv。不要为获得绿色结果删除失败包或注释依赖。

## 1. 干净构建与安装来源

```bash
cd /media/mcap_data/project_east_eleair_refactor
# 若需并行保留本轮环境，用 deploy/rollback/restore.sh HEAD /path/to/new_checkout 创建独立目录。
source deploy/env.sh s100p
python3 deploy/offline.py --check-only
./deploy/build.sh --platform s100p --plan
./deploy/build.sh --platform s100p --offline
source deploy/env.sh s100p
colcon --log-base "$TANK_RUNTIME_ROOT/log/discovery" list --base-paths "$EAST_ELEAIR_ROOT/ros2_ws/src"
./deploy/preflight.sh --json
python3 -c 'import tank_vision,inspection_runtime,d405_seam_tracking,emat_core; print(tank_vision.__file__,inspection_runtime.__file__,d405_seam_tracking.__file__,emat_core.__file__,sep="\n")'
```

判定：发现 28 包（27 原包+新适配）；profile 内包及依赖成功，排除项逐项说明；全部安装来源是新工作树/新 runtime，无旧 overlay；msg/srv/action 保持原定义；pip 只拥有一个 tank_vision 实现；tank_vision_core 来源检查通过。
检查 `.runtime/<platform>/install` 中各包 launch/config/resource、ament 索引和可执行入口；用安装后的资源运行，不以 source path 掩盖漏装。
x86/rk3588 使用对应 --platform 并提前准备同版本同架构依赖。RViz/MQTT 可选包单独用 --packages-select 显式验证；x86/rk3588 的 d405_seam_ros 默认不选，实验启用时单独构建。不能将 ARM64 wheel/deb 当作 x86 分发。
完整 native 依赖闭包、CMake configure/link/RPATH 尚未验证；D405 native BPU 另按 native/CMakeLists.txt 在 S100 的受控 SDK 下构建到 `$TANK_RUNTIME_ROOT/native/build/d405_bpu`，不写源码目录，不修改系统。

## 2. 纯软件、安装与历史资源

```bash
python3 tools/migration/validate.py --output "$TANK_RUNTIME_ROOT/validation/static.json"
python3 tools/migration/check_metadata.py
python3 -m pytest tests/operations tests/data tests/system/test_wrappers.py -q
python3 -m pytest packages/tank_vision/tests/test_config.py packages/tank_vision/tests/test_pipeline.py packages/tank_vision/tests/test_async_writer.py -q
node --check apps/web_console/console.js
node --check apps/web_console/console_core.js
node --check apps/web_console/replay.js
node --test tests/web_console/test_console_core.js
```

判定：纯计算/命令转发/退出码/终止信号测试通过；不得打开设备；JS 使用具备 node:test 的受控 Node 版本（本轮 Node 缺失）。两个未构建 ROS 模块导致的 operations/replay 测试在新 overlay 构建后重跑，不静默 skip。
视觉旧证据测试先显式设置 FIELD_VISION_EVIDENCE_ROOT 到只读原数据副本并校验封存 SHA；原 output 不整体搬入。
历史源码复现显式配置 INSPECTION_BASELINE_FIXTURES，保持旧源码仅为测试输入，绝不作为正式运行 fallback。旧 manifest 中 source_image_path/annotation_path/哈希保持历史证据语义。

## 3. 隔离 mock 数据链路与录放

```bash
export ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1
ros2 launch inspection_bringup mock_pipeline.launch.py record:=false
# 另一终端 source 同一 env 并设置相同隔离域：
ros2 topic list -t
ros2 topic hz /derived/inspection/fusion_index
```

源码固定同步输出为 /derived/inspection/fusion_index；若实际输出缺失，检查 anchor_topic 与 mock 输入契约，不改 topic 来适配测试预期。判定：mock 数值、单位、frame_id、header 时间戳、同步缓存和丢帧计数符合原契约。
完成无录制 mock 后，在已挂载数据盘新建验证 run；`<NEW_MOCK_BAG>` 是必须替换且不存在的目录：

```bash
ros2 launch inspection_bringup mock_pipeline.launch.py record:=true output_directory:=<NEW_MOCK_BAG>
# 有界采集后 Ctrl+C 正常关闭，确认 metadata.yaml 与 MCAP 尾部完整：
ros2 bag info <NEW_MOCK_BAG>
ros2 run inspection_tools validate_bag <NEW_MOCK_BAG>
ros2 bag play <NEW_MOCK_BAG> --clock
```

判定：没有真实设备节点；MCAP 可重新打开，类型/时间戳/计数及同步语义一致；退出不残留采集进程；输出仅在新验证目录。
针对已有 FIELD-VEHICLE run，另使用 `deploy/replay.sh RUN_ID core-only`、`deploy/validate.sh <RUN_DIR> --report-dir <NEW_REPORT_DIR>`，不把 mock bag 冒充完整 vehicle run。使用 `deploy/handoff.sh --run-dir <RUN_DIR> --profile common --manifest-only --output-root <NEW_REPORT_DIR>` 与 verify-handoff 验证交付，保持原文件哈希不变。
Rerun 补齐原固定依赖并构建新独立环境后运行 `ros2_ws/src/functions/visualization/inspection_rerun/scripts/build_run_rrd_nice.sh <FINALIZED_RUN>`，判定 reader 对齐、RRD manifest 和 frame-map 证据有效；不伪造缺失帧。

## 4. 网页与通信

```bash
ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 python3 -m pytest ros2_ws/src/functions/communication/agv_bridge/test/test_inspection_bridge.py -q
python3 -m http.server 8081 --bind 127.0.0.1 --directory apps/web_console
```

bridge 测试自身使用隔离模拟发布者和临时端口；普通网页 smoke 只提供静态文件，不运行 deploy/run.sh 启动硬件。浏览器验证资源无 404，连接地址可设为隔离 bridge；消息协议、断线/过期/失败状态、录像列表与回放控件不变。`tests/web_console/test_mock_workflow.py --help` 后按其参数接入准备好的 mock 服务，不将真实系统服务当 mock。
网页点云流、未接入设备与通用融合仍是明确缺口，不能用演示数据宣称已实现。

## 5. 定位与标定

```bash
ros2 launch fast_livo mapping.launch.py --show-args
# 审阅实际输入 bag 类型及冻结标定后，隔离域内再执行 mapping + ros2 bag play。
```

判定：CustomMsg/PointCloud2 的实际消息类型与预处理匹配；无重复发布者；单位、TF、时间戳及轨迹没有迁移引起的变化；Log/PCD/Colmap 输出在 runtime/fast_livo。已有粗外参 production_valid=false，不以稳定画面替代标定精度验收；恢复三张原证据或重新标定后再作精度结论。

## 6. 完整离线回归与实机

依赖、安装和冻结证据齐备后运行 `deploy/verify_offline.sh`；逐项检查新 verification.json，不复用历史报告的 verified 字段。它包括 ROS mock，未在本轮执行。
实机另行安排：先设备枚举/权限/SDK 架构与 ABI，再逐设备有限时长采集，再统一 run/collect/stop，最后录放、网页、定位联合运行。检查 Ctrl+C/异常退出是否释放唯一设备所有权，EMAT 未标定状态是否仍如实显示，磁盘满/断线/时间跳变是否拒绝假成功。
不自动安装 udev、驱动、服务，不升级依赖。每项记录命令、提交、平台、配置/依赖 SHA、输出路径、退出码；结果分别标为实际通过、依赖阻塞、失败、未执行、硬件未验证。
