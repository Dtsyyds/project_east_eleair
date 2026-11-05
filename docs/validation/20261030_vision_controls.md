# 2026-10-30 视觉巡迹实验操作接入

本轮范围经确认：启动视觉巡迹、停止视觉巡迹、保存实验样本；连续数据复用既有采集，标注和训练后续离线进行。不新增机器人运动控制、网页训练或模型管理。

## 使用方式

正常启动 `bash system_websocket/run.sh`，打开“实时巡检 → 巡迹结果”。三个按钮使用板端声明的能力和真实运行状态启用。相机彩色、深度数据须在 3 秒新鲜度内，数据盘已挂载且空间满足原配置要求。服务已运行旧版本时需正常退出后重新启动；本轮只读检查本机 8080 端口未运行，未为验证启动真实设备。

- 启动：创建独立 `VISION-*` 会话、冻结算法配置，启动现有被动 D405 节点；收到本会话节点诊断后确认运行，不把进程创建等同于启动完成。进程已在别处运行时拒绝重复启动。
- 保存：请求下一组配对输入，使用原 EventWriter 有界后台线程写原始图像/深度、已有标定、时间戳、结果及诊断；收到持久元数据、校验文件 SHA、写入索引后才确认成功。每次巡迹最多 30 组、源时间间隔至少 1 秒，保留原限制。LOST 样本允许保存用于后续负样本/失败分析，不能伪造为人工标签。
- 停止：只停止本服务拥有的巡迹进程并关闭写入器，不停止相机或连续录制，不发送运动指令。
- 断线/刷新：不停止后台巡迹；重连只查询状态，不自动重发副作用操作。未知结果先查询状态。
- 回放：实时巡迹和历史回放互斥，网页及服务端均有检查。连续采集可以与巡迹并行。

数据位置：配置数据根下的 `vision_experiments/VISION-*/`。`session.json` 保存来源、配置 SHA 和未标注/未验收声明；`config.yaml` 为本次快照；`index.json` 是现有 `d405_dataset_v1` 格式。样本目录含 `raw.npz`、`result.json`、`diagnostics.npz`、`metadata.json` 和 `collection_reference.json`。索引样本的 `split=unassigned`、`annotation=null`。保存时有正在录制的任务则关联 `run_id`，否则为 null。

连续结果只进入现有 MCAP 录制流程，不因启动巡迹另开持续结果文件录制。录制话题增加 `/vision/d405_seam/result`、`/vision/d405_seam/path_3d`、`/vision/d405_seam/diagnostics`；未改变采集状态机、预检条件和停止操作。

## 结果通路与风险控制

采用 Color + Depth，与现有 D405 彩色预览同源；配置仍为原实验 `fused/classical/cpu`，未改算法阈值。节点输出原图像尺寸和会话信息，操作服务将结果转换为已有 `tracking_result` 字段，通过现有 `/inspection/operations/events` 和通信桥发送。保留源帧坐标系与纳秒时间戳，按桥接实际预览尺寸及 OpenCV 像素中心约定转换叠加坐标；不同来源拒绝适配。未修改 C++ 桥接、JPEG 刷新、图像算法或现有控制门禁。

新增 `vision.start/stop/save_sample/status` 为原协议内的受限动作，不接受客户端路径、shell、模型或算法参数。后端按请求 ID 去重并串行化视觉副作用操作；写盘异常、缺流、超时、进程退出不冒充成功。只接受当前会话诊断和结果，独立/旧节点回包不能冒充本次运行。

样本复制仍仅发生在显式保存请求，保留原有队列容量和保存上限；不逐帧复制原始数据到样本写入队列。新增的运行控制不重新占用相机、不新增应用依赖。相机标定缺失时原算法的降级行为保留，样本元数据也保留缺失，文件保存完成不等同于三维可用。

## 本轮修改清单

| 文件 | 修改 |
| --- | --- |
| `apps/web_console/index.html`、`theme.css` | 现有巡迹侧栏添加三个按钮、操作反馈和展开后的实验路径/说明；保留 V1 主布局。 |
| `apps/web_console/vision.js`（新增） | 能力、状态、新鲜度及请求门禁，保存反馈和路径展示。 |
| `apps/web_console/console.js`、`setup.py` | 接入控件、状态查询、超时处理、回放互斥与资源打包。 |
| `packages/inspection_runtime/src/inspection_runtime/vision.py`（新增） | 被动进程管理、输入/存储检查、样本校验与索引、结果坐标适配。 |
| `packages/inspection_runtime/src/inspection_runtime/operations.py` | 新增四个受限动作，复用请求去重/工作线程/状态广播，生命周期关闭。 |
| `ros2_ws/src/functions/communication/inspection_operations/inspection_operations/operations_node.py`、该包 `package.xml` | ROS 诊断/结果订阅、样本服务调用，以及已有 D405 ROS 包依赖。 |
| `ros2_ws/src/functions/perception/d405_seam_ros/d405_seam_ros/node.py` | 会话来源、源尺寸、样本请求/查询服务；修正迁移后事件写入器的错误相对导入。 |
| `packages/d405_seam_tracking/src/d405_seam_tracking/writer.py` | 增加样本请求状态跟踪，区分等待、写入、持久完成和失败；原写入算法不变。 |
| `ros2_ws/src/bringup/inspection_bringup/config/record_topics.yaml` | 增加三项巡迹输出录制。 |
| `tests/operations/test_vision.py`、`test_vision_ros.py`（新增） | 文件校验、未标注索引、限流/异常、权限和生命周期、实际隔离 ROS 链路。 |
| `tests/web_console/test_vision_controls.py`（新增）、`mock_console_server.py` | 两种真实视口的控件工作流，新增本地脚本服务路由。 |
| `README.md`、`apps/web_console/USAGE.md`、本报告及 UI 原报告追加记录 | 使用方法、目录与验证边界。 |

所有既有未提交修改保留，未提交或推送。修改前关键文件副本位于 `/tmp/vision-controls-baseline`。

## 实际验证

- `d405_seam_ros`、`inspection_operations`、`inspection_bringup` 三个 ROS 包构建完成（5.59 秒），安装节点/配置与源码核对。构建出现既有 sysroot underlay 无 `local_setup` 的提示，但本轮三个目标均成功。
- 第一组业务、相机调参与 D405 核心回归：**73 passed（4.46 秒）**。
- 补充异步写盘失败测试后，视觉业务、实际隔离 ROS、既有 D405 集成检查：**16 passed（22.72 秒）**。两组含重叠用例，不累加为独立测试总数。
- Firefox 视觉按钮、原五页回归、V1 告警与图像测试：**8 passed，1 deselected（57.98 秒）**。排除项为可选长时间 soak。
- 原 JavaScript 状态/权限/帧匹配检查：**8 passed**。
- 最终补充“停止超时保留进程句柄、允许重试”的处理与回归后，视觉业务及实际隔离 ROS 检查：**10 passed（3.92 秒）**；无硬件隔离环境时 ROS 专项明确跳过。
- [安装来源核对](../../.runtime/s100p/validation/vision_controls/installed-sources.json)及[最终源码哈希](../../.runtime/s100p/validation/vision_controls/final-source-hashes.json)已保存。
- `git diff --check` 通过。

ROS 测试使用域 **187**、`ROS_LOCALHOST_ONLY=1` 和合成彩色/深度/标定消息：真实 ROS 服务、已安装 D405 子进程、算法输出、后台文件写入和数据索引均参与。测试确认停止巡迹后输入发布仍继续，样本 NPZ 可由现有读取器校验并读取、停止后 writer manifest 完整。存储挂载检查仅在测试对象中替换，所有样本写入 pytest 临时目录。测试没有真实传感器或运动指令；详见 [ros-workflow.json](../../.runtime/s100p/validation/vision_controls/ros-workflow.json)。该证据不是实际 WebSocket 浏览器到硬件的整机验收。

浏览器采用回环 mock，明确标注 `mock` 和“非硬件证据”。验证启动/保存/停止、录制期间停止巡迹后录制仍继续、断线禁用、重连不重发启动，以及无脚本错误/控件溢出。截图为 Firefox 实际内容视口、DPR=1、缩放 100%、`full:false`，不是长截图：

- [1366×768](../../.runtime/s100p/validation/vision_controls/browser/controls-running-1366x768.png)
- [1920×1080](../../.runtime/s100p/validation/vision_controls/browser/controls-running-1920x1080.png)
- [浏览器工作流记录](../../.runtime/s100p/validation/vision_controls/browser/workflow.json)

截图记录各自工作流当时的样本计数与任务状态，不宣称两张为相同数据状态的像素对比。主相机空白来自测试 fixture 未启用工业相机，与实际设备状态无关。

复现主要检查：

```bash
source deploy/env.sh s100p
python -m pytest -q tests/operations/test_vision.py tests/operations/test_operations.py tests/operations/test_camera_tuning.py packages/d405_seam_tracking/tests/test_core.py
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 python -m pytest -q tests/operations/test_vision_ros.py packages/d405_seam_tracking/tests/test_integration.py
PYTHONPATH=packages/inspection_runtime/src /tmp/industrial-ui-testenv/bin/python -m pytest -q -s tests/web_console/test_vision_controls.py
```

原浏览器回归输出本轮通过 `ui.OUT` 等模块变量重定向到 `vision_controls/regression/`，没有覆盖旧 UI 证据。Node 使用环境现有 VS Code 附带版本，没有安装新依赖。

## 未验证与限制

未启动真实硬件、未执行真实焊缝精度/长时间负载验收、未发送机器人运动指令。D405 USB 停流保持原状态；缺流时启动/保存会明确失败。模型训练、人工标注工具网页化、机器人坐标外参和运动闭环不在本轮范围。已有保存上限不会自动删除数据，达到上限需结束本次巡迹，再启动新会话。

## 追加：D405 专用实验采集方案（2026-10-30）

用户确认：算法巡迹实验和后续训练数据采集只保存 D405 相关数据。新增独立方案，不修改 UI V1、不切换现有三相机入口，也不改变巡迹算法、图像刷新、控制门禁、停止语义或录制状态机。

### 使用入口与范围

```bash
bash system_websocket/run_d405.sh
```

实际链路：专用脚本 → `apps/web_console/run.sh` → `deploy/run.sh` → CLI/supervisor；传入显式 `--config config/communication/d405_experiment.yaml`。操作服务子进程继承同一配置。网页使用原采集 profile 能力显示“D405 算法实验采集”。若旧服务仍在运行，必须先正常停止任务并退出，不能靠刷新网页切换方案。

- 只启动 D405；关闭工业相机、鱼眼、雷达、涡流、同步节点和粗略机器人外参发布。沿用原 D405 序列号与 640×480×15 诊断参数，没有调整 USB 或驱动。
- 预检继续使用现有存储挂载/空间与图像新鲜度逻辑，仅要求 D405 彩色、深度、Infra1、Infra2 四路图像。不读取工业相机参数。
- 连续录制使用板端配置的 14 个明确话题：四路图像及四份 camera_info、三组相机内部外参、三路已有巡迹输出。网页请求不能扩展清单，不追加 EMAT、全局 TF、全局 diagnostics 或其他设备数据。
- `run.json` 增加实际选择的 `record_topics`；旧方案不配置清单时仍使用原全量话题及 EMAT 结果，回归确认其顺序和内容不变。
- 不启动巡迹也能采原始连续数据；需要算法结果和独立样本时另点“启动视觉巡迹”。单个实验样本沿用 Color + Depth，双红外保留在连续 MCAP 中。算法结果不作为人工标签。
- supervisor 将明确停用的工业相机/鱼眼预览话题置空，复用已有 `not_configured` 呈现。没有隐藏告警或将 WebSocket 连接推断为设备正常。

风险分析：只改变本方案的启动设备、必需数据范围与录制白名单。存储保护、失流停止、正常 SIGINT 关闭、MCAP 文件尾及话题计数检查沿用原实现。所有四路原始流仍持续录制，不改变采样频率/算法以降低负载；真实性能和掉流仍须实机验收。配置的清单不保证标定实际到达，操作手册要求结束后核对标定、外参及结果计数，现有完成门禁不被夸大为训练数据质量验收。

### 修改文件

| 文件 | 本轮改动 |
| --- | --- |
| `config/communication/d405_experiment.yaml` | 新增完整独立配置和 14 话题白名单 |
| `system_websocket/run_d405.sh` | 新增专用启动入口，原 run.sh 保留 |
| `packages/inspection_runtime/src/inspection_runtime/operations.py` | 使用本地录制清单、校验其有效性、记录清单元数据；旧配置保留原行为 |
| `packages/inspection_runtime/src/inspection_runtime/supervisor.py` | 生成桥接配置时排除已停用的相机预览输入 |
| `tests/operations/test_d405_profile.py` | 预检、范围、非法配置、旧方案兼容、启动入口隔离测试 |
| `tests/operations/test_d405_recording_ros.py` | 隔离 ROS 的实际录制、算法运行、样本关联、源时间戳读取验证 |
| `docs/system/d405_experiment_collection_guide.md` | 更新为 D405 专用采集操作说明，明确启动与检查步骤 |
| `README.md`、`apps/web_console/USAGE.md`、`config/README.md` | 补充入口与配置边界 |
| 本报告、原 UI 验证报告 | 追加结果及索引，不改历史验收结论 |

### 实际测试

1. `test_d405_profile.py` + `test_operations.py` + `test_vision.py` + `test_camera_tuning.py`：**55 passed，1.35 秒**。无硬件，验证 D405 四流预检、存储限制、非法清单拒绝、浏览器不能扩充话题、旧整机/三相机录制清单保持不变等。
2. `test_d405_recording_ros.py`：**1 passed，最终 8.10 秒**。使用 ROS 域 **187**、`ROS_LOCALHOST_ONLY=1`，合成 D405 四路图像/标定与同时存在的无关话题；未启动设备驱动。真实已安装操作节点、算法子进程、rosbag2 MCAP 写入/关闭和读取器参与。
   - 连续录制最终完整；MCAP 实际话题集合恰好等于 14 项白名单，每项计数均大于零。
   - 启动录制前只发布一次的三组 transient-local 外参均录入，验证晚启动的 recorder 能收到保留的标定消息。
   - 同域发布的工业相机、鱼眼、测厚、涡流和机器人状态测试话题未录入。
   - 顺序读取 MCAP 全部消息，四路图像原始源时间戳与 fixture 发布集合一致、frame_id 保留、图像字节长度正确。
   - 实验样本可由现有读取器校验，关联本次真实录制 run_id；停止录制时巡迹仍运行，停止巡迹后测试输入继续发布。
   - 仅测试对象替换存储挂载判断，数据写入 pytest 临时目录；不是现场硬件证据。详见 [隔离录制证据](../../.runtime/s100p/validation/d405_profile/ros-recording.json)。
3. 复用 `test_vision_controls.py`：**1 passed，7.39 秒**。Firefox 回环 mock，固定 1366×768、1920×1080 内容视口、DPR=1、100% 缩放。启动/保存/停止、录制与巡迹独立、断线禁用和重连不重发通过；运动请求 **0**。
   - [1366×768 固定视口](../../.runtime/s100p/validation/d405_profile/browser-regression/controls-running-1366x768.png)
   - [1920×1080 固定视口](../../.runtime/s100p/validation/d405_profile/browser-regression/controls-running-1920x1080.png)
   - [浏览器工作流](../../.runtime/s100p/validation/d405_profile/browser-regression/workflow.json)
   - 截图是既有 mock 控件回归，非 D405 硬件或专用方案整机画面验收；各自样本计数随工作流推进，不作为相同状态的像素对比。新目录保存证据，未覆盖历史 UI 截图。
4. `bash -n system_websocket/run_d405.sh`、`bash system_websocket/run_d405.sh --help`、文档本地链接/命令语法、`git diff --check` 通过。help 确认真实入口到达 supervisor 后退出，没有启动设备。当前环境导入的 operations/supervisor 均指向本工作树的 `packages/inspection_runtime/src/`，无需重建 ROS 包；启动新进程后生效，未重启现场设备。[相关源码哈希](../../.runtime/s100p/validation/d405_profile/source-hashes.json)已保存。

复现核心检查：

```bash
source deploy/env.sh s100p
python -m pytest -q tests/operations/test_d405_profile.py tests/operations/test_operations.py tests/operations/test_vision.py tests/operations/test_camera_tuning.py
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 python -m pytest -q tests/operations/test_d405_recording_ros.py
```

### 尚未验证

未执行真实 D405 启动/长期录制、真实焊缝定位精度、真实设备到浏览器的专用方案全链路或离线训练质量验收。现有 D405 USB 停流问题保留，没有宣称已解决。没有新增应用依赖，没有发送机器人运动指令，没有提交或推送。操作者应按更新后的操作手册先完成一组真实短流程及数据核对，再扩大采集。
