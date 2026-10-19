# 罐体巡检操作台使用说明

本文说明当前目录中的轻量离线操作台如何部署、连接和操作。当前交付由以下文件组成：

- `index.html`：操作台页面和样式。
- `console_core.js`：状态、新鲜度、帧匹配、能力门禁等公共逻辑。
- `console.js`：WebSocket 通信、任务、事件、巡迹、设备与调试界面。
- `tests/mock_console_server.py`：与 ROS 和真实执行器隔离的本地 mock。
- `tests/test_console_core.js`、`tests/test_mock_workflow.py`：逻辑和协议工作流测试。

当前工作区没有板端工程、ROS 启动文件、正式部署目录和历史 `final_report.json`。因此本文中的真实板端部分只描述页面所需契约；真实设备、ROS topic、采集脚本、安装副本一致性和算法效果仍需在完整工程及现场环境中验收。

## 1. 安全边界

1. 页面启动后不会自动连接，也不会自动发送运动、采集、停止或保存请求。
2. 旧协议只使用已有的自检和预览命令。板端没有通过 `capabilities` 明确声明的新功能，页面保持禁用。
3. 网页断线、刷新或关闭不应停止板端采集。重新连接后，页面只自动查询只读任务状态，不自动重发开始、停止、保存或运动命令。
4. “发送零速度”不是硬件急停。真实急停必须使用经过验收的独立硬件或板端接口。
5. 隔离 mock 不导入 ROS，也不会发布底盘 topic。mock 顶部会明确显示“模式 mock”。
6. Token 只保存在当前页面内存中；断开或连接重建时清除，不写入 `localStorage`、日志或导出文件。

## 2. 页面部署

### 2.1 现场部署

继续沿用现有工程的静态 HTTP 部署方式，把下列三个文件放在同一目录：

```text
index.html
console.js
console_core.js
```

不要只替换其中一个文件，也不要创建 `index(1).html` 等并行页面。部署前后可用下面的命令核对三个文件是否一致：

```bash
sha256sum index.html console.js console_core.js
```

本工作区当前 SHA-256 为：

```text
cd4ca43557b32874dd026be85c74e94a664907cccff89066d4872c7b52f6ced0  index.html
8cd20388b26bb2e02a886c015e6be48c06ba18e74326e1a8b12a1eb6cb272d27  console.js
d4fa7c2b698b9a1556c01e4cc17dc1d390e8a5933808b6ed5510f0d671e0150e  console_core.js
```

上述 SHA 只代表当前独立工作区，不能代替完整工程 source/install 一致性证明。

### 2.2 本机只读预览

在当前目录执行：

```bash
cd /home/dts/websocket_console
python3 -m http.server 8080 --bind 127.0.0.1
```

浏览器打开：

```text
http://127.0.0.1:8080/
```

页面不依赖 CDN 或公网资源。不要直接以 `file://` 打开；使用 HTTP 可避免浏览器对脚本和 WebSocket 的额外限制。

## 3. 连接板端

1. 打开“设备与调试”。
2. 在“板端地址”中填写完整 WebSocket 地址，例如 `ws://192.168.1.20:9001`。
3. 按需勾选“自动重连”。
4. 只有地址不包含 Token、用户名和密码时，才可勾选“记住非敏感地址”。
5. 点击“连接”。

也可以用查询参数预填地址，但页面仍不会自动连接：

```text
http://127.0.0.1:8080/?ws=ws%3A%2F%2F192.168.1.20%3A9001
```

HTTPS 页面必须使用 `wss://`。浏览器会阻止 HTTPS 页面连接普通 `ws://`，操作台也会在发送前拒绝这种组合。

连接成功后页面先发送旧协议兼容的 `check`。如果板端随后在状态包中提供 `capabilities`，任务、事件、历史和底盘相关按钮才会按能力、权限及实时状态启用。顶部机器人身份默认是 `tank_inspection_robot`，板端能力信息提供有效 `robot_id` 后会更新来源。

## 4. 顶部常驻状态

顶部始终显示：

- 机器人身份及身份来源。
- WebSocket 连接状态。
- `live`、`mock`、`simulation` 或 `replay` 运行模式；未提供时显示“未知”。
- 当前 `run_id`。
- 采集状态。
- SSD 挂载和剩余空间。
- 当前最高级别活动告警。

“WebSocket 已连接”只代表传输通道存在，不表示设备、ROS 数据、算法、采集或正式验收正常。

## 5. 实时巡检

### 5.1 视频预览

鱼眼和 D405 面板分别显示：

- `receive FPS`：最近 2 秒内通过协议校验的预览包接收频率。
- `display FPS`：最近 2 秒内成功解码、尺寸校验并提交到画布的频率。
- 最后接收帧和当前显示帧各自的 `frame_id`、十进制字符串 `stamp_ns` 和本地时刻。
- 解码失败、尺寸不匹配、客户端待处理帧替代、隐藏页未解码、源时间乱序以及板端 `preview_dropped`。

页面只保留“一帧正在解码 + 一个最新待处理帧”。接收快于解码时，中间待处理帧会被最新帧替代，并记入“客户端替代”；不会创建无限解码任务。

图像内容按板端提供的 `content_type`、`image_type`、`semantic` 或 `source_encoding` 显示为“彩色”“Infra1”“深度伪彩”“融合叠加”等。页面不会仅凭 `camera_id=depth_camera` 把画面称为深度图。

深度 JPEG 仅供观察。没有原始深度和有效标定时，页面不显示毫米测距。分辨率相同也不表示图像和几何已对齐。

断线或超过该流的新鲜度后，最后一帧可以保留，但画面中央会覆盖“画面已过期”和最后显示时间。

### 5.2 巡迹结果

页面显示板端算法结果，不在浏览器重新实现巡迹。支持以下原始状态：

- `FUSED_CANDIDATE`：融合候选。
- `VISION_2D`：二维视觉有效。
- `GEOMETRY_CANDIDATE`：几何候选。
- `CONFLICT_2D`：二维冲突。
- `PREDICTED`：预测结果。
- `LOST`：巡迹丢失。

“候选”不会改名为“已确认焊缝”。二维有效、三维有效、身份确认和预测标志分别显示。

只有同时满足下列条件，边界和中线才会叠加到图像：

1. `robot_id`、来源、流、`frame_id` 和 `stamp_ns` 与当前显示帧完全一致。
2. 结果坐标明确为 `image_pixels`。
3. 结果声明的图像尺寸与显示帧一致。

否则页面显示“结果待匹配/已过期”“坐标不支持”或“尺寸不匹配”，不把最近收到的结果画到当前帧上。使用“隐藏叠加/显示叠加”查看原图与 overlay；画布使用原始像素坐标，CSS 缩放不改变几何坐标。

### 5.3 位姿与轨迹

位姿必须同时具备：

- 有限的 `x/y/z/angle`。
- `position_unit=m` 和 `angle_unit=rad`。
- 有效 `frame_id`。
- 十进制字符串形式的 `stamp_ns`。
- 独立、未过期的位姿记录。

轨迹是有界的轻量二维观察图，不传输全量点云。未完成资产坐标注册时，页面明确显示“未注册到罐体”，不显示 Course、Plate 或罐体缺陷位置。

### 5.4 涡流预览

只有板端真实提供 `channel_mean` 时才绘制通道均值。图表采用有界窗口，并以每个显示像素的 min/max 画线以保留均值序列中的尖峰。它不是原始波形，不影响原始录制；单位未提供时不标注 mm。

## 6. 采集任务

### 6.1 预检

1. 打开“采集任务”。
2. 填写任务名称、罐体/样件 ID、焊缝段 ID 和备注。
3. 从板端提供的 profile 列表中选择采集 profile。
4. 点击“执行预检”。

页面只提交业务字段，不提交目录或 shell 命令。`run_id` 和真实目录必须由板端生成并校验。

预检结果应由板端真实检查并返回，包括：挂载点、写权限、空间、必要设备和 topic、配置/标定版本、活动任务及 profile 条件。未配置设备应返回“未配置”，不能统一报成故障。挂载不存在时，板端必须阻止采集，不能在系统盘创建同名目录。

### 6.2 开始采集

预检状态为“就绪”且板端声明 `collection.start` 后，“开始采集”按钮启用。点击后页面显示：

- 请求 `accepted/running/completed/failed/unknown` 状态。
- 板端任务真实状态。
- `elapsed`、写入量、剩余空间。
- MCAP 和视频 writer 状态。
- 板端实际提供的队列、频率、丢帧和错误信息。

按钮点击后会立即锁定重复副作用请求。请求超时显示“结果未知”，页面先发送只读 `collection.status` 查询，不自动重发开始采集。

关闭预览不会停止录制；网页掉线也不应停止板端录制。

### 6.3 停止并封存

只有任务状态为“采集中”时，“停止并封存”按钮启用。点击后应依次观察：

```text
停止中 → 校验/封存中 → 完整完成 / 未完整 / 失败
```

页面分别呈现请求 ACK、writer 关闭、MCAP 有效、源文件 SHA 和算法/硬件验收状态。“ACK 已接收”或“进程已退出”不能单独显示为“完整完成”。没有算法或硬件验收证据时显示“未知”，不自动补成通过。

## 7. 事件标记与证据

打开“事件与数据”，选择事件类型并填写备注。支持：

- 焊缝丢失。
- 边界偏移。
- 深度异常。
- 疑似表面异常。
- 传感器异常。
- 自定义。

三个操作含义不同：

- “保存当前显示帧”：只有板端明确声明 `displayed_frame_cache=true` 且页面已有显示帧时可用。页面发送该帧的精确 `frame_ref`。
- “请求保存下一可用处理帧”：请求板端在下一可用处理帧上建立证据，不声称与当前画面相同。
- “添加事件标记”：保存人工事件和备注；如果存在当前显示帧，会附带其引用，但不把浏览器截图冒充板端原始证据。

`accepted` 只表示请求被接受；`queued/running` 表示仍在处理；只有板端返回 `complete`、文件清单和 hash 状态后才算证据完整。缺少原图、原始深度、overlay、几何或标定引用时，事件列表会显示缺失项。

“确认已读”只改变本页告警阅读状态，不表示故障恢复，也不会删除板端证据。

## 8. 历史任务与交接

板端声明 `history.list` 后，可在“事件与数据”中点击“刷新历史”。页面只显示板端登记索引返回的内容：

- `run_id` 和采集时间。
- 传感器、时长和大小。
- 封存、校验和硬件验收状态。
- 板端提供的下载服务说明、路径提示或交接命令。

板端绝对路径只作为提示，不会生成 PC 本地可点击文件链接。已封存源 run 应保持只读；后加备注和分析结果应作为派生内容引用源 run。

## 9. 设备与调试

物理设备和数据流分开展示：

- “物理设备”只列鱼眼、Mindvision、D405、涡流、EMAT、底盘和激光雷达。
- `eddy_raw`、`eddy_mean`、`lidar_pose` 等显示在“ROS 数据流与质量”，不会重复算作硬件。

每个对象独立记录更新时间。局部设备更新不会延长其他设备、位姿、巡迹、控制或存储状态的新鲜度。状态区分“未知、未配置、已禁用、活跃、已过期、设备错误、桥接断开”。新鲜度优先使用板端 `freshness_ms`、`expected_hz` 或 `expected_period_ms`；板端未提供时使用按对象类型区分的保守前端阈值。

“最近共享/关联回包”中：

- 带 `request_id` 的新协议回包可与本客户端请求关联。
- 不带 `request_id` 的旧 `response` 明确标为共享/未关联，不能作为某个按钮的独立 ACK。

清空页面日志只清理浏览器内最多 200 条的显示记录，不删除板端事件、任务或文件。

## 10. 底盘单次命令验证

本轮只保留受控的单次命令验证，不提供连续遥控或自动驾驶。必须同时满足：

1. WebSocket 已连接。
2. 板端声明 `chassis.move` 和 `chassis.stop`。
3. 当前会话具有权限。
4. 板端最新 `control.enabled=true` 且未过期。
5. 板端声明支持的自由度和每轴限速。
6. 操作员显式勾选“启用本页面命令测试”。

页面只显示板端声明的轴。例如板端只声明 `vx/wz`，`vy` 输入会隐藏。页面不会默认底盘支持 `vy`。

Token 如需使用，在发送时加入协议包，但在日志和导出中脱敏。回包中的“已接收”“ROS 已发布”和“硬件反馈”必须由板端分别提供；未知执行结果不能显示成功。

## 11. 掉线、隐藏页面与恢复

- 自动重连使用递增退避，最大等待 5 秒。
- 每次连接重建都会重置视频 FPS 和计数窗口，旧连接异步解码结果不会提交到新连接。
- 断开时清除 Token 和本页控制开关。
- 页面隐藏时保留最新一个待显示帧，不继续堆积解码；恢复后只处理最新帧。采集任务不受影响。
- `pagehide` 会关闭 WebSocket 和停止渲染定时器；从浏览器后退缓存恢复的 `pageshow` 会只重建一个定时器，并按自动重连设置恢复连接。
- 重连后只恢复任务状态等只读查询，不自动重发采集、停止、保存或运动操作。

浏览器与板端时钟没有经过可验证同步时，页面只报告本地“接收年龄”，不把源时间差称为端到端延迟。

## 12. 隔离 mock 验证

mock 只用于验证软件交互和异常状态，不能作为真实硬件、算法精度或 30 分钟现场稳定性证据。

### 12.1 启动

终端 1：

```bash
cd /home/dts/websocket_console
python3 tests/mock_console_server.py --host 127.0.0.1 --port 18765
```

浏览器打开：

```text
http://127.0.0.1:18765/?ws=ws%3A%2F%2F127.0.0.1%3A18765%2Fws
```

进入“设备与调试”，点击“连接”。顶部必须显示“模式 mock”。结束后在终端 1 按 `Ctrl+C`。

### 12.2 正常工作流

1. 确认双路预览有独立 receive/display FPS。
2. 确认 D405 显示“深度伪彩”，不是笼统的“深度相机画面”。
3. 在“采集任务”执行预检。
4. 预检就绪后开始采集。
5. 停止并观察“校验/封存中”到“完整完成”。
6. 在“事件与数据”请求保存下一帧，确认先出现队列/处理中，再出现 complete 和缺失 `raw_depth`。
7. 刷新历史，确认 mock 条目标明硬件未验收。

协议级自动测试在 mock 运行时执行：

```bash
python3 tests/test_mock_workflow.py --base http://127.0.0.1:18765
```

该测试覆盖旧共享回包、预检、开始、重复开始幂等、停止封存、证据完成、历史查询，并断言底盘请求数为 0。

状态逻辑测试不需要启动 mock：

```bash
node tests/test_console_core.js
```

### 12.3 切换异常场景

以下命令只改变隔离 mock：

```bash
curl -X POST http://127.0.0.1:18765/mock/scenario/lost
curl -X POST http://127.0.0.1:18765/mock/scenario/conflict
curl -X POST http://127.0.0.1:18765/mock/scenario/disk
curl -X POST http://127.0.0.1:18765/mock/scenario/finalizing
curl -X POST http://127.0.0.1:18765/mock/scenario/event_failure
curl -X POST http://127.0.0.1:18765/mock/scenario/corrupt
curl -X POST http://127.0.0.1:18765/mock/scenario/dimension
curl -X POST http://127.0.0.1:18765/mock/scenario/partial
curl -X POST http://127.0.0.1:18765/mock/scenario/normal
```

场景含义：

| 场景 | 预期页面表现 |
|---|---|
| `lost` | 巡迹显示 LOST 和失败原因，形成告警 |
| `conflict` | 显示 CONFLICT_2D，不把候选称为确认结果 |
| `disk` | 顶部 SSD 未挂载，显示写盘阻断告警 |
| `finalizing` | 当前任务显示校验/封存中 |
| `event_failure` | 证据先排队，随后显示写盘失败 |
| `corrupt` | D405 解码失败计数增加，旧画面最终标记过期 |
| `dimension` | D405 JPEG 尺寸不匹配计数增加，不提交显示 |
| `partial` | 仅鱼眼局部更新；D405、位姿等分别过期 |
| `normal` | 恢复正常 mock 数据 |

### 12.4 查看 mock 是否触碰底盘

```bash
curl http://127.0.0.1:18765/mock/state
```

自动化验证后 `chassis_requests` 必须为 `0`。

## 13. 页面与板端操作映射

| 页面动作 | 协议消息 | 能力要求 |
|---|---|---|
| 查询设备自检 | `msg_type=check`, `payload={}` | 旧协议已有 |
| 开启双路预览 | `msg_type=command`, `camera.command=start_stream` | 旧协议已有 |
| 停止双路预览 | `msg_type=command`, `camera.command=stop_stream` | 旧协议已有 |
| 执行预检 | `action=collection.preflight` | `capabilities.operations` 明确声明 |
| 开始采集 | `action=collection.start` | 同上；板端负责幂等和 run 生成 |
| 停止并封存 | `action=collection.stop` | 同上；应返回可查询 job/任务状态 |
| 刷新任务 | `action=collection.status` | 同上；只读 |
| 保存当前显示帧 | `action=evidence.save_displayed` + `frame_ref` | 操作能力 + `displayed_frame_cache=true` |
| 保存下一处理帧 | `action=evidence.save_next` | 操作能力 |
| 添加事件标记 | `action=event.mark` | 操作能力 |
| 刷新历史 | `action=history.list` | 操作能力 |
| 单次速度 | `action=chassis.move` | 操作、权限、控制状态、轴和限速全部有效 |
| 零速度 | `action=chassis.stop` | 同上；不是急停 |

所有新操作都带 `request_id` 和会话内 `client_id`。页面只允许板端声明的 `command` 或 `check` 消息类型，不接受能力信息把页面变成通用远程 shell。

## 14. 常见问题

### 按钮一直不可用

先查看按钮下方原因，再到“设备与调试 → 能力协商”核对板端是否声明对应 operation。仅连接成功或勾选本页开关不足以获得写盘或运动权限。

### 收包 FPS 正常但显示 FPS 为 0

查看视频面板计数。常见原因是 JPEG 损坏、回包尺寸与 JPEG 实际尺寸不一致、页面隐藏，或新连接已经替代旧连接。接收成功不等于成功显示。

### 看到旧图像

断线或流过期后页面保留最后一帧用于定位，但中央会显示“画面已过期”。元数据始终对应已提交画布的帧，不会使用最新回包给旧图贴时间和几何。

### 停止后为什么不是“完整完成”

停止 ACK、writer 退出、MCAP 有效、SHA 校验和算法/硬件验收是不同状态。板端没有完成或没有提供某一项时，页面保持“封存中”“未完整”“失败”或“未知”。

### 请求超时后能否再点一次

不要立即重复副作用操作。页面会把结果标成 `unknown` 并优先查询任务状态。确认板端没有执行后，再按现场规程决定是否重新操作。

### 清空日志是否会删除证据

不会。它只清理浏览器当前页的日志 DOM 和内存记录，不改变板端任务、事件或文件。

## 15. 真实环境待验收清单

在完整 `/home/sunrise/east_eleair` 工程或其真实维护副本恢复后，仍需完成：

- 核对适用 `AGENTS.md`、实际协议实现、桥接节点、采集/停止/finalize 和 snapshot/failure recorder。
- 把页面 action 映射到既有受限实现，验证服务端权限、幂等、任务登记、路径约束、有界队列和背压。
- 核对真实 topic、encoding、图像语义、标定、时间映射和 freshness 配置。
- 核对源码与安装副本 SHA，并使用现有安装/回退流程部署。
- 在隔离 ROS domain/namespace 做软件联调，在现场单独完成硬件、算法和时长验收。
- 将实际证据写入现有 `final_report.json` 的独立阶段字段，不由浏览器把正式生产状态设为通过。

建议分别维护：`console_software_verified`、`legacy_protocol_compatible`、`collection_workflow_verified`、`snapshot_workflow_verified` 和 `hardware_field_verified`。
