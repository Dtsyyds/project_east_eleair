# 构建、安装与回退

执行 README 的 build.sh。构建使用 explicit --base-paths，防止总控备份源码被 colcon 当作重复包；COLCON_IGNORE 保护总控目录。build_cpp/install_cpp、build_physic/install_physic、build_vision/install_vision、build_core/core_env 与原 build/install/log 隔离。仅构建实际受影响包及依赖闭包，没有升级 SDK。

core setup.py 从权威 pyproject.toml 读取 metadata。当前 setuptools 59.6 不原生处理 PEP621，必须保留此兼容层，不能宣称原 UNKNOWN wheel 可用。pip --no-deps --no-build-isolation 使用现有依赖；环境/版本冻结于 environment.json。

验收分纯 Python、ROS C++/实际 E2E、Rerun 三类。cpp colcon 实际注册9个测试；最初自动发现备份导致0 tests的失败日志保留，不计成功。原 MCAP 失败已从 traceback 复核为插件缺包。本次通过离线 sysroot 提供 additive 插件，没有升级 rosbag2 core；实际 roundtrip、24 项 RRD 和多 bag DDS 已通过，历史失败报告仍保留。最新证据见 final_report.json.closeout。

回退只需新终端 source 原 `/opt/ros/humble/setup.bash`、`physic_bev/install/setup.bash`、`tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash` 和原 Python 环境，使用原入口及安装代码；不要 source 本目录 env.sh。若原采集脚本保留源码注入，设 FIELD_USE_INSTALLED_CODE=1 并使用原已安装 core 环境，避免加载本次改过的源码。FIELD_PHYSIC_INSTALL/FIELD_TANK_INSTALL 可显式指定原安装。任务不执行 Git reset，也不覆盖现有安装。

本次源码依然保留所有用户预先修改，编辑前备份在 reports/refactor_v1/backups；备份不能直接当工作空间参与构建。当前兼容证据不等于硬件6Hz或真实数据完整验收，production_ready=false。


已实际生成基线 core wheel 和只含原编辑文件副本/未编辑文件链接的 rollback 入口；它们是备份安装产物，不作为正常源码仓库。切换命令：

```bash
# 新终端；以下 help 不会启动设备
S=/home/sunrise/east_eleair/tank_inspection_system
bash "$S/scripts/rollback.sh" --help
bash "$S/scripts/rollback.sh" validate --help
# 用户需要回退采集时，替换为原参数；此命令将启动真实采集，本任务未执行
bash "$S/scripts/rollback.sh" collect run_id:=新的回退采集ID
```

rollback.sh 选原 ROS installs、core_env_baseline 与 pre-task 脚本。新机可先运行 `python3 scripts/prepare_rollback.py` 重新生成已有备份的安装工件。新 frame_map 字段和文档不是原版本能力；回退恢复原异常/宽松验收行为，不能将其视为修复后的严格通过。

新增无线仅构建 agv_protocol/agv_bridge，复用现有 eddy_driver/inspection_interfaces 安装依赖，build_wireless/install_wireless 保留当前产物。agv_protocol 的 websocketpp 由系统或 offline/sysroot 查找，缺失明确失败，不访问 GitHub。原两个机器人 ID 和入口保留；未知 ID 现在拒绝，退出只停止本实例拥有的 legacy launch 进程组，移除了全局 pkill 行为。core 构建直接从权威源生成 wheel 并检查 67 个 Python 文件 SHA，不再维护 staging/staging_v2 源码副本；baseline_staging、core_env_baseline 与 rollback 仍保留。

31 个本地依赖归档的 SHA 与版本登记于 offline/dependency_manifest.json。新 wheel 是 websockets 测试客户端的匹配 aarch64/Python3.10 版本；已安装同版本无需重装。MCAP 原 core 0.15.15 与新增 plugin/vendor 0.15.17 的 ABI 已实际验证；不能用插件注册代替本次读写/CDR证据。现有 SDK 不升级，离线缓存不是裸机 OS/Humble 安装介质。

同版本 wheel 的安装曾被 pip 跳过：隔离 venv 中先安装同版本旧内容，再执行普通 pip 可复现文件未更新。install_core.py 现在使用 --force-reinstall --no-index --no-deps；新增 --venv 可指定隔离安装目标。已实际验证临时目标的 67 个文件和 installed CLI，当前 core_env/回退环境未覆盖。正常使用本机环境无需重装已存在的 websockets；若该测试依赖确实丢失，可从匹配归档离线恢复：

```bash
"$S/core_env/bin/python" -m pip install --no-index --no-deps \
 "$S/offline/wheels/websockets-15.0.1-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl"
```
