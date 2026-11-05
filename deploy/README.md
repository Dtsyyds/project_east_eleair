# 部署入口

## RDK 一条命令 + 电脑网页控制

入口：`bash system_websocket/run_robot.sh --real --host RDK_IP`。保留老的 `run.sh`、`run_d405.sh`，不强迫相机单独运行场景启动底盘。整合入口不复制或维护第二份底盘驱动源码，而是复用已构建的 `Duct_Robot_V2_Ros2Control_humble` 工作区。

正式启动脚本始终放在本工程 `system_websocket/run_robot.sh`。网页扩展版独立命名为 `web_console_chassis_tracking`：优先读取本工程 `apps/web_console_chassis_tracking`，否则读取与本工程同级的 `web_console_chassis_tracking`；不会回退去修改或使用下载原版 `web_console`。也可通过 `ROBOT_WEB_ROOT` 或 `--web-root` 指定网页目录。扩展版目录里的同名脚本仅为兼容转发，不保存另一套启动逻辑。

首次更新先退出旧 launch、旧网页桥接和键盘发布节点；不要并行运行两套硬件控制。将新版 `web_console_chassis_tracking` 整个目录放到上述位置，再更新本工程 `tools/robot_session.py`、`tools/robot_gateway.py`、`system_websocket/run_robot.sh`，不要覆盖原版 `web_console`，也不能只复制 HTML。旧服务若占着 9001，先正常退出本工程旧终端，再运行整合入口，不能两个进程直接争用同一端口。本工程 `.git` 仍指向旧 RDK worktree，本次没有擅自修复 Git 或上传。

还必须把底盘工作区的 `src/duct_robot_web_bridge` 和 `src/duct_robot_tracking` 更新到 RDK，再构建；新消息接口及其他包沿用前面已更新的完整版本。不要仅复制网页：

```bash
cd /home/sunrise/Desktop/Robot_Chassis_Code/robot_chassis_ros2ctrl
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  duct_robot_msgs duct_stm32_driver duct_stm32_driver_node \
  duct_robot_hardware duct_robot_description duct_robot_tracking \
  duct_robot_web_bridge duct_robot_bringup
```

从任意目录执行（IP 按现场替换）：

```bash
bash /media/mcap_data/project_east_eleair_refactor/system_websocket/run_robot.sh \
  --real --host 192.168.1.148
```

入口加载环境，启动底盘、循迹、底盘桥接、模块录制、现有 D405 candidate 相机上位机和通信路由。**原对外端口保持不变：网页 `http://RDK_IP:8080/`，WebSocket 通信 9001。** 相机 `/ws/inspection`、底盘 `/ws/robot` 都使用原通信端口 9001，不把网页入口改成 9001，也不新增对外端口。后台仍独立运行：底盘桥接绑定 `127.0.0.1:9002`，相机 WS 使用 9003，相机后台 HTTP 使用 8081，但网页不需分别填写这些地址。启动时从原相机配置生成临时运行副本，只调整内部转发端口并固定资源路径，不覆盖源 YAML 或算法参数。`--http-port` 表示原网页端口（默认 8080），`--port` 表示原通信端口（默认 9001），`--camera-http-port` 是后台内部端口（默认 8081）。相机 `deploy/env.sh` 的平台默认 `s100p` 沿用当前工程，可 `--platform rk3588` 指向已经准备的相应依赖，不会自动安装 SDK。

主页加载即连接两条同端口 WS，顶部显示底盘连接状态；不必进入底盘页。主页面的“断开”同时断开底盘连接并撤销其运动授权，人工“连接”会一起恢复连接。网络断开后自动重连，但不重新勾选手动开关、不重发运动命令、不再次启用循迹。相机断流与底盘断流分开显示，不把一侧失联冒充另一侧已正常。统一网关只转发固定的本机服务，不提供任意地址代理；任一连接关闭立即关闭对应上游，原桥接租约和停车保护保留。旧 `ws://IP:9001`、`/ws` 仍作为相机协议别名兼容；网关部署后旧浏览器记住的直连端口不再覆盖统一默认地址。

常用选项：

- 不加 `--real`：循迹预览模式，不能让车自动循迹；手动控制仍可在明确勾选后使用。
- `--tracking-params /绝对路径/weld_tracking.yaml`：沿用现场 PID、置信度和超时；不会重置为默认数值。只有横向 Kp 时保持 Ki/Kd/heading_kp 为 0。
- `--camera-mode cameras`：沿用三相机入口，而非默认 D405 candidate。
- `--no-camera`：仅启动底盘与统一网关；相机/视觉由其他完整部署提供。如果还需要相机网页流，其后台 WS 必须与 `--camera-port`（默认 9003）一致，不能占用网关的 9001。
- `--chassis-workspace /实际工作区`、`--data-root /数据盘目录`：修改部署与结果位置。
- `--mcap-modules chassis,tracking,driver`：只录关注模块，减少进程与磁盘开销；默认 all。
- `--plan`：只显示命令，完全不启动；`--check`：只检查环境和端口，不连接设备。

网页操作顺序：原实时巡检页面启动视觉识别 → 新页面确认相机误差/置信度/图像年龄 → 点击“启动循迹”。无需 CLI enable。先“停止循迹”，再勾选手动控制；先松开手动控制并取消勾选，再启动循迹。停止后的 DDS 发布者释放约需 100ms，图发现还有延迟，页面若提示先释放手动控制，等待后重试，不自动重放启动。探头 1/2 独立占空比控制；没有编码器，不声称闭环转速。

网页关掉、刷新或连接丢失，桥接停止该连接持有的自动控制；再次连接不会自动运动。单纯切回相机页面会释放手动底盘和探头，但自动循迹可以继续，便于观察相机。首次使用请在安全条件下低速验证，软件零速不是硬件急停。遥控 CH11 自动交还需要此前已更新的 0x3C 固件/消息；缺权限反馈时不得绕过安全门禁，本次不修改或烧录固件。

MCAP 位于 `DATA_ROOT/robot_bags/会话目录/`，每个模块一个包，根目录 `INDEX.md`、`recording_manifest.yaml` 提供结果与缺失话题。网页“录制与诊断”只读展示最近清单，明确标注本次/历史会话与模块消息数；录制中消息数须收尾后统计，清单状态不是实时写盘验收。相机大图像仍由原采集任务独立录制。启动日志位于 `.runtime/平台/robot_sessions/时间/` 下的 `robot.log`、`camera.log`、`gateway.log`。终端 Ctrl+C 只停止本次进程组，允许 MCAP 收尾，不使用全局 pkill；录制结果以清单为准。

当前本地副本相机后台多个 `.py` 为空，完整相机运行尚未验证；不能用占位文件替换 RDK 原来可工作的后台。已实施的测试是新模块的隔离 ROS/WebSocket、启动计划和浏览器验证，不是现场硬件验收。500ms 是可配置的调度容忍，不代表解决之前 RDK 硬件缓存卡顿的根因；原有故障锁存/图像超时仍然保留。

所有主入口可从任意当前目录通过绝对路径调用，`--help` 不启动设备。先阅读 README 与下一轮验证计划。

- 日常上位机启动：项目根目录执行 `bash system_websocket/run.sh`，自动加载环境并默认选择当前三相机配置，无需参数。
- env.sh：source 设置本工作树与平台路径；拒绝继承原工程路径的环境；已有系统 ROS underlay 可读使用，不安装或修改系统。
- build.sh：--plan 只打印命令；默认构建 profile，--packages-select 实际按依赖闭包 packages-up-to 选择；--skip-native 使用明确准备的依赖；--offline 校验原清单并在新 sysroot 解包 ARM64 deb。所有 build/install/log/venv/native/sysroot 在新 `.runtime/<platform>`。
- run.sh / apps/web_console/run.sh：启动硬件、operations、bridge 和网页 supervisor；`--config config/communication/cameras.yaml` 选择三相机模式。默认整机配置仍检查涡流；启动前先阅读最新实机报告。
- collect/stop/replay/validate/handoff：转发 inspection CLI；run_id、name:=value 等保持原能力，子命令帮助可查看入口语法。
- preflight.sh：仅检查依赖/工具/资源，新 ROS overlay 缺失返回 2；不是硬件预检通过证明。
- verify_offline.sh：迁移后的离线/ROS mock 回归工作流；显式设置 `FIELD_VISION_EVIDENCE_ROOT` 指向冻结视觉证据目录后运行。输出独立验证目录，不修改历史报告。
- rollback/restore.sh COMMIT NEW_DIRECTORY：创建独立 detached worktree 后按对应版本重建；不 reset 当前工作、不复用旧 install。
- offline.py --check-only：校验 33 个原清单归档，不解包。

pip 安装步骤去掉 PYTHONPATH，避免旧 setuptools 在源码已可见时省略 editable .pth 导致干净环境不可导入。部署仍使用项目固定源码；不安装旧业务 wheel。
Rerun 使用独立 `.runtime/<platform>/rerun_venv`（FIELD_RRD_PYTHON 可明确覆盖），保持 0.36.2 与其 NumPy2 需求隔离，不污染 D405 的 NumPy1.26.4。S100P 环境已建立并通过 MCAP/RRD 回归；其他主机仍须准备对应版本依赖。tools/vision/setup_rerun_s100p.sh 仅从本地 wheel 安装。
兼容脚本列表见 docs/architecture/compatibility_manifest.json。旧 --wireless-only 转译到新 build --packages-select agv_bridge。
