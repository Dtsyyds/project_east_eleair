> 2026-11-10 启动方式更新：当前统一使用 `bash system/run_robot.sh`，SLAM 页面服务默认启动，已移除 `--slam` 和页面“独立打开”入口。`run_slam.sh` 仅用于整机内部启动；下文独立启动和可选接入方案为历史设计，当前操作以根 README 和上位机 USAGE 为准。

# 实现内容：新增独立定位建图上位机，并接入现有主界面

新增 `apps/web_console_slam/`，提供直接建图、重定位建图、结束时保存地图、完整启动日志和实时有效点云显示。通过 `system_websocket/run_slam.sh` 独立运行，通过原整套入口新增的 `--slam` 参数接入主界面。

**状态：已完成软件实现与断电后续做。独立后端、页面、主界面嵌入和整套可选启动已落地；软件验证记录及文件清单见 [修改.md](../../apps/web_console_slam/修改.md)。真实输入、直接建图和先验重定位仍需现场验收。**

工程根目录为 `/home/fire/Desktop/project_east_eleair`。下文树中的路径均相对此目录；部署到 RDK 后按实际工程位置解析，不写死开发机路径。

## 1. 本版结构与启动方式

本版替换此前“复制整套主界面到 `host/`”的方案。复用现有 `web_console_chassis_tracking` 主界面，只增加一个子页面入口；SLAM 的前后端和测试集中放在新目录中。

```text
浏览器访问 RDK:8080（现有整套上位机）
├── 实时巡检、采集任务、底盘与循迹等现有页面
└── 定位与建图
    └── iframe → 同一 RDK:8082
                  └── 独立 SLAM 后台 → 已有定位建图 launch

浏览器也可直接访问 RDK:8082，使用同一份 SLAM 页面与后台。
```

| 入口 | 行为 |
|---|---|
| `system_websocket/run_slam.sh` | 只启动独立 SLAM 服务；已有雷达与 IMU 输入由外部提供 |
| `system_websocket/run_robot.sh --slam` | 原整套会话增加 SLAM 服务，主界面新增入口可直接使用 |
| `system_websocket/run_robot.sh` | 保持原启动内容；定位建图入口显示独立服务的连接情况 |

- 两种方式启动的都是同一个后台入口。服务启动后处于空闲状态，点击网页“启动”才运行算法。
- 第一版统一使用 `8082` 同时提供 SLAM HTTP 和 WebSocket，避免新增一套端口配置同步机制。现有 `8080/8081/9001/9002/9003` 不变。
- 独立服务不导入 `inspection_runtime`、不调用 `inspection_operations` 或 `agv_bridge`。原整套启动仍使用原有巡检后台。
- 独立服务不启动或停止传感器；整套模式下传感器继续由原启动链负责。两种方式均不重复启动驱动。
- 不再创建 `host/`、`run_integrated.sh`、应用内重复的 `run.sh/env.sh` 或整套前端资源副本；不复制算法源码、launch 和基础 YAML。

## 2. 文件结构与修改范围

```text
/home/fire/Desktop/project_east_eleair/
├── apps/
│   ├── web_console_slam/                      [新增，功能主体]
│   │   ├── index.html                        页面结构
│   │   ├── console.js                        控件、连接、日志及 WebGL 点云显示
│   │   ├── theme.css                         本页面必要样式
│   │   ├── config/slam.yaml                  算法资源、输入、运行目录及超时
│   │   ├── pyproject.toml                    独立 Python 包和依赖声明
│   │   ├── slam_console/
│   │   │   ├── __init__.py
│   │   │   ├── __main__.py                   CLI 入口、配置与启动预检
│   │   │   ├── server.py                     HTTP/WS、状态和请求分发
│   │   │   ├── session.py                    会话、launch、日志及地图结果
│   │   │   ├── ros_bridge.py                 ROS 输入检查、状态与点云订阅
│   │   │   └── pointcloud.py                 算法输出 PointCloud2 的 XYZ 解析
│   │   └── tests/                            独立后端和页面功能测试
│   └── web_console_chassis_tracking/
│       ├── index.html                        [修改] 导航、iframe 和嵌入策略
│       ├── slam_embed.js                     [新增] 少量父页面接入代码
│       └── USAGE.md                          [修改] 补充两个启动方式
├── system_websocket/run_slam.sh               [新增] 唯一 SLAM shell 入口
├── tools/robot_session.py                     [修改] 可选 SLAM 子进程与退出策略
├── tests/operations/test_robot_session.py     [修改] 整套启动与故障隔离测试
├── docs/system/web_console_slam实现方案.md    本方案及实施后的说明
└── .runtime/<platform>/slam_console/          运行时生成，沿用现有 Git 忽略规则
```

不要为了占位再创建独立的集成管理器、配置管理包或日志服务。上述模块各自承担实际职责；页面先保持一份 JS，后续确因体量需要再拆分。

### 新目录之外分别修改什么

- `apps/web_console_chassis_tracking/index.html`：增加“定位与建图”导航、`view-slam` 区域、iframe、连接提示和独立打开链接；加载 `slam_embed.js`。调整 CSP 的 `frame-src`，允许同一设备的 SLAM 页面。其余页面布局和操作保持现状。
- `apps/web_console_chassis_tracking/slam_embed.js`：使用浏览器当前访问的主机名构造 `8082` 地址，不能写成浏览器本机的 `localhost`；负责 iframe 加载、就绪提示和页面可见性通知，不承接 SLAM 业务逻辑。`postMessage` 必须校验来源窗口及准确 Origin，不直接读取跨端口 iframe 的 DOM。
- `system_websocket/run_slam.sh`：按自身位置解析工程根目录，复用 `deploy/env.sh` 加载已有 ROS/安装环境，再加载本模块并 `exec python -m slam_console`。复用环境初始化不代表要求巡检业务包可用；独立预检不检查它们。此脚本提供 `--platform`、`--domain`、可重复的 `--host`、`--config`、`--plan`、`--check` 和 `--help`。
- `tools/robot_session.py`：增加 `--slam` 和 `--slam-config`；启用时调用上述唯一脚本，传递平台、ROS 域和允许访问的主机。把它加入启动计划、静态预检、会话日志和清理流程，不能再编写一套整机 supervisor。
- `apps/web_console_chassis_tracking/USAGE.md`：只补充独立启动、整套启动、地址及外部输入要求，详细说明链接到本文件。
- `tests/operations/test_robot_session.py`：验证可选启动、原默认行为、故障隔离与保存等待时间。SLAM 模块本身的测试放在新目录内部。

主界面通过独立的 `8082` 服务加载 SLAM 页面；新增的父页面 JS 由现有网关静态文件服务提供。

## 3. 页面与交互

页面名称为“定位与建图”，从上到下排列：

1. **启动前配置区**：模式、先验地图、解析后的地图路径，以及全部勾选项集中在这里。模式为“直接建图 / 重定位建图”；先验地图由后台枚举配置目录内的 PCD 文件，直接建图时禁用该选择。“结束任务时保存地图”默认勾选，任务开始后锁定本次启动配置。
2. **大号启动/停止按钮与状态**：按钮最小高度 64px、宽度 240px，窄屏满宽。空闲时绿色“启动”；请求发出后至运行期间变红并显示“停止”；再次点击停止本次任务。停止和保存期间灰色禁用，显示“正在停止…”；旁边区分停止、保存、失败等状态。
3. **启动日志框**：显示本次建图和重定位 launch 的完整 stdout/stderr，标明来源和时间；提供跟随、暂停跟随、清空显示及历史加载。清空只影响页面，不删除后台文件；这些控件使用按钮，不在配置区之外散放勾选项。
4. **建图可视化**：日志框下方显示三维点云，支持有效点云/配准点云切换、最近 N 帧、旋转、缩放、俯视、复位和清空。显示话题、frame_id、帧时间、点数和数据新鲜度。

- 启动请求尚未返回时再次点击“停止”，记录取消意图；取得本次会话标识后停止该会话，不能停止其他任务。
- 刷新、切页、关闭浏览器或断线不停止后台算法。重连先查询状态，不重放启动命令；状态未确认时显示未知，不假定已经停止。
- iframe 只是同一独立页面的展示方式。切走该页可暂停绘制，后台算法、日志与保存继续运行；父页面的巡检连接按钮不控制 SLAM。
- SLAM 服务未运行时，主界面显示未连接、重试和独立打开入口，其他页面可继续使用。

## 4. 后台与算法要求

### 输入、模式和就绪判定

- 原始输入沿用 `/livox/lidar`，类型 `livox_ros_driver2/msg/CustomMsg`；IMU 为 `/livox/imu`，类型 `sensor_msgs/msg/Imu`，加速度必须已转换为 m/s²。现有 IMU 适配器或等价上游均可，不以包名判断兼容性，也不能从话题名推断单位。
- 仅消费已有输入。启动按钮执行预检，检查话题类型、新鲜数据、时间戳、地图和输出目录、依赖及冲突算法实例。保留当前算法的系统时间检查，不自动关闭检查或修改时钟。
- 配置默认引用本工程 FAST-LIVO2 源码根目录和已安装 `fast_livo` 的基础 YAML；允许显式指定其他已构建算法位置。路径基于配置或已解析的算法根目录，不依赖终端当前目录。
- 每次启动生成独立运行 YAML，不覆盖原配置。直接建图设置 `locate_in_prior_map=false`；重定位建图设置 `true` 且 `prior_map_update_en=true`，表示在先验中重定位后继续扩图。
- 两种模式均调用 `relocalization/slam.launch.py`，复用建图与辅助子 launch，共用本次 YAML，统一管理建图、求解器和 TF 三节点，关闭 RViz；此入口不启动传感器驱动。
- 运行副本保持 `common.img_en=0`、`preprocess.lidar_type=1`，并开启 `publish.pub_effect_point_en=true`；其他算法参数沿用基础配置。
- 直接建图初始化超时默认 60 秒；重定位到可运行总超时默认 180 秒。使用本次启动之后的新鲜里程计和 `tracking_valid=true` 判定就绪，重定位还必须收到本次 `initial_map_pose`；进程存活和固定延时不代表就绪。
- 重定位求解器完成配准且初值被接受后取消看门狗、释放输入订阅与服务客户端，保持空闲存活，让 FAST-LIVO 继续初始化与建图；直接建图时求解器从启动即停用配准。统一 launch 中任一核心节点退出即收尾整组算法，不再依赖求解器成功后退出的旧约定。算法异常和初始化超时只停止本次 SLAM 任务，服务保留状态与日志供查看。
- 页面提示静止初始化；不自动下发底盘运动。SLAM 与 MCAP 录制分别控制，不因停止 SLAM 而停止录制。本版不通过巡检后台增加全局回放锁；实时使用时不得同时向相同 ROS 输入发布历史回放数据。

### 结束与保存

- 启动时将本次保存选项同时写入 `pcd_save.pcd_save_en`、`pcd_save.pcd_save_en_relocate`，保持 `interval=-1`。沿用算法退出保存能力，不新增运行时保存服务。
- 输出分别使用算法根目录下 `PCD/priori/<session_id>/` 或 `PCD/relocate/<session_id>/`，每个 session_id 唯一，防止算法分钟级文件名前缀造成覆盖。保存坐标类型沿用基础配置的模式自动选择。
- 默认先验目录为 `PCD/map`，由后台限制选取范围并返回解析后的绝对路径；输出不覆盖先验文件，不自动提升为下一次先验。
- 停止先发送 SIGINT，允许算法保存并收尾；统一 launch 向内部节点传递 `sigterm_timeout:=120`、`sigkill_timeout:=5`，并在目标 ROS Humble 上验证生效。后台清理预算至少 130 秒，整套启动器为 SLAM 保留 150 秒，不能套用原统一 12 秒强杀期限。
- 结束后检查本次目录产物和进程结果，区分“保存成功 / 未启用保存 / 无有效地图 / 保存失败 / 保存未确认”。不因进程退出就报告保存成功；若算法仅生成一种合法 PCD，不强求一定存在降采样文件。
- Ctrl+C 使用本次已经锁定的保存选项。未启用地图保存时仍保留日志、运行配置及会话记录。

### 独立接口、日志和点云

- HTTP 和 `/ws` 共用 `8082`。独立 JSON 协议提供 `maps.list`、`preflight`、`start`、`stop`、`status`、`logs.read`；启动参数为模式、地图名称和保存布尔值。请求带 request_id，响应及会话事件带 session_id，停止明确针对指定会话。控制请求做有界去重；日志读取和点云不放入控制结果缓存。
- 后台只接受固定操作和配置目录内地图，不接受网页传入 shell 命令、任意 launch 名称或任意文件读取路径。按 `--host` 限制允许的浏览器来源；独立页面允许由对应主界面嵌入。
- 后台直接通过 `rclpy` 订阅算法状态及 `/cloud_effected`、`/cloud_registered`；两者仍是 PointCloud2。原始 CustomMsg 适配已经完成，本方案不重新修改驱动或增加格式转换节点。
- XYZ 解析支持 float32/float64、大小端及行填充，过滤非有限坐标。每个话题最多推送 5Hz、每帧最多 2000 个点；页面默认保留最近 30 个收到的预览帧，N 可选 1～100，总显示点数不超过 200000。慢客户端只保留最新预览，不能阻塞算法或积压无界队列。
- 默认为有效点云 `/cloud_effected`。会话、话题或 frame_id 切换时清空历史；直接显示实际消息坐标系，不擅自将 `camera_init` 标为 `map`。显示空帧、过期和解析异常，不修改 ROS 原始消息及录包内容。
- 为本次所有 launch 持续读取完整 stdout/stderr，分别保存原始日志，同时生成带来源和时间的 `launch.log`；停止时读尽剩余输出。页面按游标每批最多读取 64KiB，支持断线续读和历史分页，不能只保留最后 200 行。
- 浏览器以纯文本呈现日志，处理 UTF-8 分块和 ANSI 控制序列；显示缓冲有界且可继续读取磁盘历史。日志正文不进入周期状态包，清空显示后游标仍能继续工作。

会话运行目录统一放到现有忽略的 `.runtime/<platform>/slam_console/<session_id>/`，至少包含 `config.yaml`、`session.json`、各 launch 原始日志和 `launch.log`。不要在源码目录新建另一套 runtime 忽略规则。

Python 包名使用独立的 `slam_console`，发行名采用唯一名称，不复制旧 `inspection_web` 包。依赖声明沿用项目已验证的 `numpy==1.26.4`、`PyYAML==6.0.3`、`websockets==15.0.1`；ROS 消息和 `rclpy` 使用 ROS 安装环境。复用既有部署环境，不自动联网安装或升级依赖。

## 5. 整套进程管理与兼容

- 默认不带 `--slam` 时保持原有启动计划、预检及退出行为；不要求 SLAM 算法已构建。
- 带 `--slam` 时静态预检增加独立服务依赖、8082 端口及配置检查；此时传感器可能尚未启动，不能把实时输入检查放在整机启动之前。实时输入在点击 SLAM 启动时检查。
- `--plan` 只打印命令和配置位置，不启动服务、传感器或算法，不要求 ROS/设备在线；`--check` 检查软件和端口，不启动硬件。独立入口也保持这一区分。
- 整套运行期间 SLAM 服务异常退出只记录一次故障，不停止底盘、相机或原网关，不自动重启算法。SLAM 内部任务失败尽量保留服务在线，以便读日志和重新操作。
- 原关键子进程退出、整套入口 Ctrl+C 或正常结束时，必须同时收尾本次拥有的 SLAM 服务及算法；对原有子进程保留原超时，只对 SLAM 使用更长保存窗口。
- 服务和算法均禁止脱离可管理的进程树；父子层级都记录本次拥有的进程/进程组，并确保服务异常退出或超时后不会遗留算法。清理只能针对本次进程，不使用宽泛的 `pkill`。
- 已有独立 SLAM 服务占用 8082 时，新的独立或整套启动明确报占用，不接管、不杀死原服务；可保留已运行服务，另用普通 `run_robot.sh` 打开主界面。
- ROS 域默认 77，整套模式与原会话一致；使用其他域时沿用现有整套入口的限制。独立模式允许显式指定，不加载底盘工作区作为硬依赖。
- FAST-LIVO2、重定位及其原生依赖需提前在目标环境构建安装。

## 6. 实施顺序与验收

按“独立后台 → 独立页面 → 现有主界面接入 → 整套可选启动”的顺序实施。先保持 `run_slam.sh` 可独立验证，再接入原启动器，避免把模块可用性绑到整套巡检环境。

至少验证以下场景：

1. 原 `run_robot.sh --plan` 与默认行为保持现状；加 `--slam` 后仅多出独立服务，同域运行且不重复启动驱动。两个入口的 `--help/--plan/--check` 不启动硬件。
2. 独立运行不要求导入巡检业务包或加载底盘工作区；8080 嵌入和直接访问 8082 使用同一份页面、同一后台会话，网线/无线地址都正确，CSP 不阻止 iframe。
3. 两种模式、无先验地图、地图不存在、输入类型错误、数据过期、非 SI IMU、时钟检查失败和外部算法冲突均有可理解的结果；禁止覆盖基础 YAML 或已有地图。
4. 启动后按钮变色并显示停止，快速二次点击可以取消，运行时配置锁定；重定位求解器成功后空闲存活，建图继续初始化并输出有效定位；初始化超时只收尾 SLAM。
5. 保存勾选/取消、没有有效点、输出无权限、写盘失败和同一分钟连续两次任务；Ctrl+C 保存与页面停止一致。用耗时超过 12 秒的保存进程验证不会被原整机期限截断，并在 Humble 上验证 launch 内部的升级信号时限。
6. 超过 200 行日志、双 launch 交错、stderr、中文跨分块、ANSI、长行、停止末尾输出及断线续读；页面可读全量历史，清空显示不删除日志，控制响应不因日志或慢客户端阻塞。
7. 两类算法 PointCloud2、浮点类型/大小端/行填充、NaN、空帧及切换；N 帧和总点数限额生效，隐藏页面暂停绘制且后台持续运行。
8. 刷新、关闭 iframe、断网和重连不停止或重复启动算法；SLAM 故障不结束其他模块，整套退出能完成本次 SLAM 保存且无残留进程。

硬件未在线时完成单元测试、模拟 ROS/子进程及浏览器验证；真实 CustomMsg、米制 IMU、直接建图和实际先验重定位必须在目标 RDK 验收。模拟通过不能代替真实算法验证。

## 7. 手动测试：整套上位机与独立 SLAM

以下 SLAM 命令已实现。示例采用当前 RDK 工程路径；每个新终端先设置 `SLAM_TEST_ROOT`，按实际位置调整。服务启动后保持空闲，网页点击“启动”才运行算法。

### 7.1 测试整套上位机

在 RDK 终端依次执行。`--plan` 查看启动内容，`--check` 通过后再正式启动：

```bash
SLAM_TEST_ROOT=/media/mcap_data/project_east_eleair_refactor

bash "$SLAM_TEST_ROOT/system_websocket/run_robot.sh" --slam --plan
bash "$SLAM_TEST_ROOT/system_websocket/run_robot.sh" --slam --check
bash "$SLAM_TEST_ROOT/system_websocket/run_robot.sh" --slam
```

启动计划应包含原有 robot、camera、gateway，以及一个 slam 服务。首次测试沿用默认循迹预览模式；需要测试实车循迹时再在启动命令末尾加 `--real`，SLAM 功能不依赖该参数。

电脑浏览器打开可达地址：

- 网线：`http://192.168.127.10:8080/`
- 无线：`http://192.168.1.148:8080/`

按以下步骤判断通过：

1. 检查原有实时巡检、采集任务和底盘页面可以正常进入，再打开“定位与建图”。页面应连接到独立服务并显示空闲状态，初次进入不自动运行算法。
2. 选择“直接建图”，勾选“结束任务时保存地图”，点击大号“启动”。按钮变红并显示“停止”，配置锁定，日志持续输出；静止初始化完成后状态进入运行，点云和帧时间持续更新。
3. 切换有效点云/配准点云、调整 N、旋转和缩放；切到其他页面再返回，任务应继续。刷新网页后恢复同一会话状态，不额外启动算法。
4. 点击“停止”，等待保存结果，确认页面显示本次实际 PCD 路径，原有其他页面仍可使用。取消保存后再测一次，本次不应生成地图，但应有日志和会话记录。
5. 准备与现场匹配的先验地图，切换“重定位建图”，选择地图并启动；确认完成先验初始化后继续建图。缺少地图时应明确提示，不能假装成功。
6. 最后在启动终端按 Ctrl+C，等待地图和录制收尾，确认本次整套服务、SLAM 算法及其子进程均已结束。

测试期间无须另外运行 `run_slam.sh`，避免重复占用 `8082`。

### 7.2 测试独立 SLAM 上位机

先结束带 `--slam` 的整套会话，释放 `8082`。若已有可用的 CustomMsg 雷达和米制 IMU 输入，可以直接执行下方“终端 C”；从零测试时使用 A、B、C 三个终端，仅启动输入和独立页面服务。

**终端 A：只启动雷达驱动，将驱动原始 IMU 输出重映射到 `/livox/imu_raw`。**

```bash
SLAM_TEST_ROOT=/media/mcap_data/project_east_eleair_refactor
source "$SLAM_TEST_ROOT/deploy/env.sh" s100p
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0

ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=1 -p multi_topic:=0 -p data_src:=0 \
  -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame \
  -p user_config_path:="$SLAM_TEST_ROOT/ros2_ws/src/drivers/livox_ros_driver2/config/MID360_config.json" \
  -r /livox/imu:=/livox/imu_raw
```

JSON 使用现场已经验证的雷达配置，网络地址应与设备一致；MID360s 使用对应的 `MID360s_config.json`。这些输入命令复用现有节点，不需要新建驱动启动文件。

**终端 B：把 IMU 的 g 转换为 m/s²。已有等价米制 IMU 发布者时跳过此步。**

```bash
SLAM_TEST_ROOT=/media/mcap_data/project_east_eleair_refactor
source "$SLAM_TEST_ROOT/deploy/env.sh" s100p
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0

ros2 run inspection_livox_adapter livox_imu_adapter_node
```

**终端 C：确认输入并启动独立 SLAM 服务。**

```bash
SLAM_TEST_ROOT=/media/mcap_data/project_east_eleair_refactor
source "$SLAM_TEST_ROOT/deploy/env.sh" s100p
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0

ros2 topic type /livox/lidar
ros2 topic type /livox/imu
ros2 topic echo /livox/imu --once

bash "$SLAM_TEST_ROOT/system_websocket/run_slam.sh" --plan
bash "$SLAM_TEST_ROOT/system_websocket/run_slam.sh" --check
bash "$SLAM_TEST_ROOT/system_websocket/run_slam.sh"
```

话题类型应依次为 `livox_ros_driver2/msg/CustomMsg`、`sensor_msgs/msg/Imu`。静止时 IMU 加速度向量的模应约为 9.8 m/s²，不能把驱动约为 1g 的原始值直接交给算法。类型检查不代表数据持续在线，启动按钮的实时预检还需检查新鲜数据。

电脑浏览器打开 `http://192.168.127.10:8082/` 或 `http://192.168.1.148:8082/`。依次测试直接建图、日志、点云、停止保存、取消保存和先验重定位，标准同上；此时不需要打开 `8080`，也不需要启动底盘或整套巡检后台。

在终端 C 按 Ctrl+C 后，只应结束独立 SLAM 服务及它启动的算法，A、B 中的输入节点继续运行。结束全部测试时再分别停止 A、B。

### 7.3 查看测试产物

实际路径由页面状态和启动终端打印，按本次 session_id 对照：

| 产物 | 默认位置 |
|---|---|
| SLAM 会话配置、状态和日志 | `.runtime/s100p/slam_console/<session_id>/` |
| 整套启动日志 | `.runtime/s100p/robot_sessions/<启动时间>/` |
| 直接建图 PCD | FAST-LIVO2 根目录下 `PCD/priori/<session_id>/` |
| 重定位建图 PCD | FAST-LIVO2 根目录下 `PCD/relocate/<session_id>/` |

先确认页面结果、日志和真实产物一致，再判断测试通过。没有硬件时只能验证界面与模拟链路，不能据此认定真实建图通过。

如果暂时不使用 SLAM，仍可用原整套入口，不带新增参数：

```bash
SLAM_TEST_ROOT=/media/mcap_data/project_east_eleair_refactor
bash "$SLAM_TEST_ROOT/system_websocket/run_robot.sh" --check
bash "$SLAM_TEST_ROOT/system_websocket/run_robot.sh"
```


## 8. 实施说明与验证边界

- 功能主体位于 `apps/web_console_slam/`。独立后台只导入本模块、Python 依赖和 ROS 消息环境，不导入巡检业务包；`deploy/env.sh` 仅复用环境初始化。原算法、launch、驱动和基础 YAML 未修改。
- 默认 HTTP/WS 端口固定为 `8082`，父页面端口为 `8080`；`--host` 可重复指定浏览器实际访问的主机/IP，需同时用于主界面和独立入口。精确校验 Host、Origin、消息来源窗口。其他代理/HTTPS/自定义父页面端口不在本版默认部署范围。
- 本次 launch 的守护进程通过 stdin 生命周期管道和 Linux subreaper 清理拥有的后代。服务崩溃也会触发 SIGINT 收尾；整套正常退出给 SLAM 150 秒。断电无法保证内存中的地图保存，恢复服务不会自动重启上次算法，也不会把未确认的旧会话标为保存成功。
- 保存成功必须有本次目录的非空、结构完整 PCD，且未观察到进程/内部节点失败或强制终止。仅有一种合法 PCD 可以成功；失败期间已写出的可用文件仍列在结果中。`session.json` 保留内部节点错误与进程退出码。
- 原始 stdout/stderr 分别保留，`launch.log` 汇总时间和来源。页面按字节游标加载当前会话全量历史，单批 64 KiB；页面清空不删除后台数据。服务重启前的历史文件保留在各会话目录，可按 session_id 使用 `logs.read` 查询。
- 自动测试均不启动传感器或驱动底盘。Humble 验证只执行无硬件的模拟子进程；浏览器测试使用模拟状态/点云，不能替代实际点云质量、重定位和地图产物验收。

复现软件验证（使用本机已安装依赖，不自动下载）：

```bash
cd /media/mcap_data/project_east_eleair_refactor
source deploy/env.sh s100p
python3 -m pytest apps/web_console_slam/tests tests/operations/test_robot_session.py -q
# Firefox 可用且 8080/8082 空闲时运行真实浏览器验证：
SLAM_BROWSER_TEST=1 python3 -m pytest apps/web_console_slam/tests/test_browser.py -q
```

独立纯 Python 测试无需导入巡检包；同时测试原整套入口时按上面命令加载项目环境。`test_humble_shutdown.py` 要求已构建的 ROS Humble 算法安装环境，未满足时明确跳过；浏览器测试默认跳过，须显式启用。
