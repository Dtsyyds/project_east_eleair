# 2026-10-30 上位机工业视觉重构

本轮直接改造 `apps/web_console`，未新建另一套应用。现有未提交的相机接入、调参和业务修改作为工作基线保留。没有修改 ROS 节点、消息定义、设备驱动、录制实现或算法；没有以测试数据替代生产数据。

## 项目分析与实施范围

项目按 `apps`（操作入口）、`packages`（业务和算法）、`ros2_ws/src`（驱动、通信、接口与 bringup）、`config`、`deploy`、`tools`、`tests` 分层。实际 GUI 为原生 HTML/CSS/JavaScript + Canvas，HTTP 静态页面通过 WebSocket 接入现有 ROS 桥接及 operation 服务。不是 Qt/Electron 应用。

| 页面 | 现有能力 | 本次呈现调整 |
| --- | --- | --- |
| 实时巡检 | 工业相机、鱼眼、D405、精确帧叠加、巡迹、二维位姿轨迹、涡流均值、告警 | 主相机优先，辅助画面与状态侧栏，诊断展开，图例与空状态 |
| 采集任务 | 任务表单、预检、采集、停止封存、writer/校验分项 | 统一表单、阶段条、关键操作与指标层次 |
| 数据回放 | 多路影像、点云旋转/缩放、IMU、涡流/超声波形、测厚、话题记录 | 统一窗口、曲线配色、空状态和表格；保留全屏与既有回放控制 |
| 事件与数据 | 帧留证、人工标记、近期事件、历史交接、验证导出 | 证据操作与列表分区，表头固定、区域内滚动 |
| 设备与调试 | 连接、自检、物理设备、ROS 数据流、受门禁保护的单次运动、能力与日志 | 统一健康状态、连接表单、日志、控制危险区域 |

当前没有独立的实时点云、激光轮廓或完整缺陷分类工作页，本轮不添加虚构入口、图表或测量数据。现有表面异常事件、焊缝巡迹及回放点云保留。

原界面采用横向导航，顶部状态与操作拥挤，三个视频面板权重相同，长帧元数据直接占用监控空间，样式与绘图配色分散。实施顺序为结构分析 → 统一主题 → 主监控 → 其他四页 → 浏览器与业务回归。

## 视觉规范

- 背景 `#0b121c`，面板 `#141f2d`，次级面板 `#101925`；科技蓝 `#4d9fff` 和青蓝 `#63cbd5` 用于操作与数据。
- 正常绿 `#64cfaa`、警告黄 `#edbb66`、故障红 `#f18e94`，全部配文字；未知状态保留中性色。连接成功不合成为“整机就绪”。
- 公共间距以 8/12/16/20/28 px 为主，组件圆角 4–8 px，细边框、轻阴影，无渐变、发光和持续动画。
- 使用本地无衬线字体和等宽数字；本地内联 SVG 线性图标，无 Emoji、网络字体、图标库或 CDN。
- 左侧导航、顶部运行与存储状态、快捷操作栏、中央工作区、辅助信息侧栏。桌面保持状态与停止按钮可见；窄屏/低矮窗口自然滚动。
- ≥1800 px 主相机与两个辅助画面并排；中等窗口主相机在上、辅助画面在下；≤720 px 单列、横向可滚动导航。表格只在自身容器滚动。
- 图像保留原始比例；不改 Canvas 原始帧尺寸及叠加坐标。帧 ID、时间、错误计数可展开查看。
- 涡流图沿用原有有界 min/max 聚合，新增通道图例，并明示各通道独立缩放，避免误读跨通道幅值。轨迹与无数据状态不添加装饰性假数据。

## 修改文件清单

| 文件 | 修改内容 |
| --- | --- |
| `apps/web_console/index.html` | 五页结构、侧导航、常驻操作区、相机/状态布局、SVG 图标、可访问性标记、主题资源引用 |
| `apps/web_console/theme.css`（新增） | 统一颜色/字体/间距、全部组件状态、响应式与全屏样式 |
| `apps/web_console/theme.js`（新增） | 启动时读取一次 CSS 配色，为实时与回放 Canvas 共用 |
| `apps/web_console/console.js` | 复用设备判定构建主监控摘要、图例/空态、告警等级样式、过期巡迹视觉、导航当前页标记 |
| `apps/web_console/replay.js` | 复用主题曲线、背景、坐标网格、标签颜色 |
| `apps/web_console/setup.py` | 安装资源包含主题及既有 `camera_tuning.js`（原清单遗漏） |
| `apps/web_console/USAGE.md` | 更新资源部署与布局说明、视觉回归命令；说明沿用当前自动连接行为 |
| `tests/web_console/mock_console_server.py` | 隔离测试服务增加主题资源路由及 CSS MIME 类型 |
| `tests/web_console/test_visual_console.py`（新增） | 实际 Firefox 布局/交互/异常/回放渲染及可选持续刷新测试 |
| `docs/validation/20261030_ui_redesign.md`（新增） | 本报告 |

`camera_tuning.js` 在任务开始时已经是未跟踪文件，本轮没有修改它。`console_core.js` 也完全不变。其他原有工作区改动未回退或合并提交。

## 风险分析与功能保持

改动前确认以下边界：视觉改动不应改变发送命令条件、图像队列/帧身份、录像服务、权限或实时数据频率。

基于任务开始时保存的工作区文件逐函数对照：67 个函数保持原文不变，包括 `sendOperation`、`sendControl`、`controlGate`、`receive`、`validateVideo`、`receiveVideo`、`enqueueDecode`、`decodeFrame`、`drawOverlay`、`renderButtons`、`renderTask`。只改动 7 个呈现相关函数，`bindUi` 中只添加导航 `aria-current` 与切页回到顶部；没有改变原有处理器、业务状态或连接方式。

新增摘要仅复用 `assess()`，创建固定 7 个节点；不新增网络请求、定时器或数据缓存。主题颜色启动读取一次，不在解码路径读取 CSS。图例只在通道数变化时构建。未改变图像刷新周期、点云投影、曲线降采样、轨迹缓存、预览队列和隐藏页处理。既有运动开关、限速、Token、能力/权限门禁、停止指令与急停区别保留。

## 实际运行和测试结果

使用现有 Firefox Marionette 与 Node（VS Code 附带）完成检查。测试专用 `/tmp/industrial-ui-testenv` 补齐既有 mock 的 `aiohttp` 依赖，没有新增应用依赖或更改工程运行环境。

| 检查 | 结果 | 证据范围 |
| --- | --- | --- |
| `test_console_core.js` | 8 passed | 状态新鲜度、配置/禁用/错误/断线区分、精确帧匹配、图像语义、能力门禁、脱敏、安全连接、巡迹标签 |
| `test_visual_console.py`（120 秒 soak） | 4 passed，148.18 秒 | 真实 Firefox + 回环隔离 mock；不访问 ROS 或设备 |
| 最终浏览器回归（不重复 soak） | 3 passed（最终复核） | 最终样式、全屏进入/退出、采集、留证与异常场景 |
| `test_mock_workflow.py` | passed | 兼容回包、预检、幂等开始、停止封存、留证与历史，`chassis_requests=0` |
| `test_operations.py` | 21 passed | 既有操作服务隔离单测 |
| `test_camera_tuning.py` + `test_replay.py` | 首轮 14 passed、2 skipped；加载现有 ROS 环境后回放 4 passed | 参数事务/持久化、回放约束、实际 ROS 类型编解码与暂停/停止；最终相关后端单测合计 37 项通过，无遗留跳过项 |
| 脚本语法、差异空白检查 | passed | 本次脚本与 `git diff --check` |
| 安装资源打包检查 | passed | 临时 wheel 确认 index、4 个业务 JS、theme.css、theme.js 共 7 项资源 |
| 原有逻辑对照 | passed | 67 个函数、完整 core 和相机调参脚本保持不变 |

浏览器覆盖：1920×1080、1440×900、1366×768、1024×768、768×1024、390×844，五页共 30 个组合。检查页面溢出、可见按钮/输入/徽标文本宽度、重复 ID、导航激活语义；截图逐页留存。调参面板开合、叠加开关、帧诊断展开、全屏、预检/采集/停止、事件留证、查询历史、告警已读、磁盘异常、损坏帧过期与恢复、断线控制禁用均经实际浏览器操作。回放点云/波形使用仅测试渲染 API 的数据包验证，不声称读取真实 MCAP。

120 秒持续刷新：采样从已显示 4 帧增至 921 帧，DOM 节点始终 671、日志始终 9，无捕获的页面异常；既有 64×48 测试图像保持更新。这是隔离低负载短时检查，不能推导真实高分辨率图像的性能或硬件长时稳定性。

回放测试首轮因默认解释器未加载 `rosbag2_py` 跳过 2 项，随后执行 `source deploy/env.sh s100p`，在现有 ROS 环境补跑 `tests/operations/test_replay.py`，4 项全部通过，无需安装或修改运行依赖。

首次浏览器测试出现 mock 服务清理超时，原因是服务等待 WebSocket 关闭；已修正测试 fixture 在退出前断开测试连接，最终测试无该错误。

证据目录：`.runtime/s100p/validation/ui_redesign/`

- `before.png` / `after.png`：相同 1440 窗口、离线状态的改造前后对比，不使用虚构设备画面。
- `realtime-1920.png` 等：五页宽屏/窄屏截图，测试模式明确为 mock。
- `alarm-lost.png`：异常状态截图。
- `replay-panels-*.png`：仅测试渲染包的回放窗口截图。
- `responsive.json`、`soak.json`、`logic-preservation.json`：布局、持续刷新与逻辑对照记录。

## 运行方式与未完成的现场验证

现有正式入口不变：`bash system_websocket/run.sh`。本轮只启动本地静态页面和回环隔离 mock，没有启动真实硬件采集。仅查看页面可执行：

```bash
python3 -m http.server 8080 --bind 127.0.0.1 --directory apps/web_console
```

浏览器仍沿用项目既有的自动连接行为；若不运行板端，显示未连接/未知/无画面。测试命令见 `apps/web_console/USAGE.md` 第 16 节。

未解决/待现场复核：

1. 未执行真实相机持续采集、真实 MCAP 完整回放、运动执行、相机参数应用/保存及 30 分钟以上硬件运行；不能标记整机或算法验收通过。
2. 既有 D405 USB 停流问题不属于视觉范围，仍以 `20261029_camera_refactor.md` 的硬件验证记录为准。
3. 本轮真实浏览器验收为 Firefox；未在 Chromium/Windows 或现场高 DPI 屏幕做实测。CSS 提供响应式与本地字体回退，仍建议现场复核。

目前已测试范围内没有未修复的布局或交互失败。截图/测试证据与真实现场验收明确区分。

## 第二轮：实时巡检页小范围精修（2026-10-30）

本节记录在上述改造基础上的增量，不替代第一轮验证。只改了页面结构与必要样式，没有重做其余四页。

### 有效源码与基线确认

- 检索仓库（含隐藏目录、排除第三方/生成物）及 `/`、`/media`、`/media/mcap_data` 的祖先路径，未发现适用 `AGENTS.md`。
- 已阅读本报告，并查看上一轮 `.runtime/s100p/validation/ui_redesign/before.png` 和 `after.png`。
- 实际加载链路为 `system_websocket/run.sh` → `apps/web_console/run.sh` → `deploy/run.sh` → `inspection_cli run` → `inspection_runtime.supervisor`。`deploy/env.sh` 将 `EAST_ELEAIR_ROOT` 设为本工作树，supervisor 的静态服务器直接提供 `ROOT / 'apps/web_console'`。
- 页面仍引用 `theme.css`、`theme.js`、`console_core.js`、`replay.js`、`camera_tuning.js`、`console.js`；本轮改的是上述有效源目录，不是 `.runtime` 安装副本。
- 修改前将这 7 个文件复制到 `/tmp/ui-refinement-baseline`，用于相同状态截图与逐文件校验。所有已有未提交修改保留；未提交或推送 Git。

### 本轮修改清单与理由

| 文件 | 必要修改 |
| --- | --- |
| `apps/web_console/index.html` | 移除顶部及实时页重复英文眉题/介绍；右栏调整为告警→巡迹→传感器；画面类型、D405 数据限制归入原帧诊断；巡迹源信息等移入原有详情展开区。所有 170 个 DOM ID 保留。 |
| `apps/web_console/theme.css` | 收紧顶部留白；实时页设备名称 14px、状态 13px、视频按钮 13px、辅助文字 12px；主预览按可用高度适配；保留正常滚动与原始比例，移除视频卡片外壳的溢出裁切。 |
| `tests/web_console/test_realtime_refinement.py`（新增） | 复用现有 Firefox/隔离 mock，补充真正固定视口截图、基线对照、告警可见性、大尺寸图像与覆盖层缩放/全屏测试。 |
| `docs/validation/20261030_ui_redesign.md` | 追加本轮证据、测量、范围和限制。 |

公共改动限于常驻顶部区域。其余四页的页面 DOM 与本轮基线逐字一致，并已回归它们的布局与主要交互。没有新增字体、图标、动画或应用依赖。

右栏告警完整保留在可见区域，不置于折叠容器。无活动告警时沿用原有文字；活动告警同时保留顶部横幅、右栏严重等级和已读操作。短说明改为“已读不表示故障恢复”，原完整说明保留在其 `title`。巡迹状态、原因、新鲜度保持直接可见，低频详情可完整展开。

D405 的伪彩/测距限制没有删除；现在位于帧信息与诊断中。位姿和涡流仍完整保留为下方次级观察区，没有折叠或暂停更新。没有新增任何“连接成功即设备正常”的判断。

### 固定视口条件与实测布局

使用本机现有 Firefox/headless Marionette、独立新 profile、浏览器缩放 100%、`devicePixelRatio=1`、滚动位置 `(0,0)`。先补偿浏览器工具栏高度，再断言 `innerWidth/innerHeight` 为 **1920×1080 或 1366×768**。截图调用 `WebDriver:TakeScreenshot` 的 `full:false`，并解析 PNG 头断言像素尺寸与视口相同。因此以下为固定视口截图，不是整页长截图。

前后对比均为：通过同一个本地静态服务机制加载相应版本，WebSocket 地址只指向该测试静态服务器的非 WebSocket 路径，随后主动断开；连接标记“已断开”、模式未知、采集未开始、SSD 未知、三路显示帧计数均为 0。未连接真实板端。测试前后状态一致；浏览器版本、尺寸、布局与 computed style 详见 `fixed-viewport.json`。

| 项目 | 精修前 | 精修后 |
| --- | ---: | ---: |
| 顶部常驻区域高度（两个视口相同） | 222.8 px | 156.8 px |
| 主图容器顶部位置（两个视口相同） | 375.4 px | 256.6 px |
| 1366×768 主相机整块区域底边 | 877.2 px，首屏外 | 758.3 px，首屏内 |
| 1366×768 告警区域范围（空状态） | y=889.9–1001.1 px | y=206.6–280.4 px |
| 1366×768 传感器区域底边（空状态） | 664.8 px | 759.5 px；告警与巡迹已前置 |
| 主图容器高度：1920 / 1366 | 614.4 / 409.8 px | 620 / 403 px |

主图上移 **118.8 px**，顶部减少 **66 px**。1366 视口下主图高度仅轻微调整，保留图像比例，使画面与其按钮/诊断入口完整可见；没有通过压扁视频强塞一屏。1920 视口可同时看到主相机与两个辅助相机及其操作。1366 视口的辅助画面和次级图表允许正常下移；出现告警后，顶部与右栏优先展示告警，其他内容自然延长，不裁切以制造无滚动效果。

主要字号通过 `getComputedStyle` 核查，不依据截图估算。设备名 14px、状态 13px、视频标题 15px、视频按钮 13px，FPS/巡迹年龄/诊断说明 12px。以面板 `#141f2d` 为底，正文 `#e6edf5` 对比度约 **14.08:1**、辅助 `#9cacc0` 约 **7.18:1**（此处是指定文字与面板色的计算，不声称整页所有状态都经过无障碍认证）。原有按钮可用/禁用/悬停/选中配色保留，连接和故障继续有文字说明。

固定视口证据位于 `.runtime/s100p/validation/ui_refinement/`：

| 条件 | 精修前 | 精修后 |
| --- | --- | --- |
| 1920×1080，已断开、无数据 | [前](../../.runtime/s100p/validation/ui_refinement/before-disconnected-1920x1080.png) | [后](../../.runtime/s100p/validation/ui_refinement/after-disconnected-1920x1080.png) |
| 1366×768，已断开、无数据 | [前](../../.runtime/s100p/validation/ui_refinement/before-disconnected-1366x768.png) | [后](../../.runtime/s100p/validation/ui_refinement/after-disconnected-1366x768.png) |

其他固定视口截图（均明确为测试）：

- `mock-alarm-1366x768.png`、`mock-alarm-1920x1080.png`：现有隔离 mock 的 LOST 告警，顶部与右栏均可见。
- `large-mock-1920x1080.png`、`large-mock-1366x768.png`、`large-mock-1024x768.png`：测试专用三路网格图，经真实 WebSocket→解码→Canvas 路径显示，图上写明 `UI TEST / NOT HARDWARE`，顶部模式为 `mock`。
- `large-mock-fullscreen.png`：主相机全屏的测试图与覆盖层。
- `fixed-viewport.json`、`contrast.json`、`scaled-geometry.json`、`unchanged-scripts.json`：视口/状态/字号、指定颜色对比度、图像几何与生产脚本 SHA-256 记录。

### 相关回归与安全边界

执行：

```bash
PYTHONPATH=packages/inspection_runtime/src \
UI_BASELINE_DIR=/tmp/ui-refinement-baseline \
/tmp/industrial-ui-testenv/bin/python -m pytest -q -s \
  tests/web_console/test_realtime_refinement.py \
  tests/web_console/test_visual_console.py -k 'not soak'

# 使用环境现有 Node 运行，未安装新的 JS 依赖
node --test tests/web_console/test_console_core.js
```

本机 Node 不在默认 PATH，实际使用 VS Code 已附带的 Node 可执行文件执行第二条；测试命令本身不变。

- 新增固定视口/告警/大源图测试 **3 项通过**；原有浏览器主要回归 **3 项通过**（共 6 项，持续运行 soak 本轮未重复）。
- 原有状态与门禁 JS 单测 **8 项通过**。
- 原浏览器回归覆盖五页、六种窗口尺寸，包含调参面板开合、叠加、全屏、采集预检/开始/停止、留证、历史、磁盘异常、坏帧过期与恢复、断线禁用。新增测试补上真正的内容视口断言，而非仅设置外窗尺寸。
- 额外使用源尺寸 **1920×1200 / 1280×720 / 640×480** 的测试 JPEG，验证三路图像与 overlay 原始像素尺寸一致、CSS 边界一致、宽高比误差小于 0.005、图像边界位于显示区内、精确帧匹配仍为 `matched`、中线位置存在覆盖像素；窗口缩放、主图全屏及退出后均通过。
- 展开巡迹详情后内容继续刷新；下方位姿数值、涡流图例继续接收现有数据。没有增加缓存、定时器或切换刷新策略。
- 所有 5 个生产 JS 文件与本轮基线 **逐字节一致**；原有数据解码、ROI/中线绘图、ROS/WebSocket、订阅、门禁、录制状态机完全没有修改。所有 170 个 DOM ID 保留，其余四页 DOM 未修改。
- 所有隔离服务断言 `chassis_requests=0`；测试服务只绑定 `127.0.0.1` 并在 fixture 退出时关闭。本轮未向真实机器人发送运动指令，未运行正式硬件启动脚本。
- `git diff --check` 通过。

首轮固定视口测试发现主相机区域底边超出 1366×768 首屏约 20px，已通过可用高度调整修正，没有隐藏页面溢出。测试还修正了告警截图等待条件：必须等到实际 LOST 文案和告警条目出现，不仅检查可能保留的 `data-severity` 属性。

### 本轮剩余限制

没有新增未修复的已知布局失败。实际硬件图像、长时间负载、现场 DPI、Chromium/Windows 仍未验收；测试网格和 mock 不属于硬件证据。第一轮 120 秒短时检查不作为本轮重新执行的结果。D405 USB 停流保持原有待解决状态，本轮未处理。固定对比的 baseline 是精修开始时已完成第一轮改造的工作区，重跑历史对比需要保留该 `/tmp` 副本；持久化 PNG 与 JSON 证据已写入本项目 `.runtime`。

## 第三轮：冻结 UI V1，告警语义与有流检查（2026-10-30）

本轮以第二轮完成后的 1366×768、1920×1080 布局为 UI V1 基线，停止整体美化和结构调整。沿用已确认的启动/静态文件加载链路，修改有效源码 `apps/web_console`。开始前将当前七个前端文件保存至 `/tmp/ui-v1-baseline`；此基线与第二轮的 `/tmp/ui-refinement-baseline` 不同。

### 实际发现及必要修改

1. **空告警集合不能证明当前无活动告警。** 原 `renderAlerts()` 无条件将空集合显示为“当前无活动告警”，断线或从未收到告警也如此。当前 `acceptAlerts()` 按 ID 合并增量记录，空数组/字段缺失不会清空历史；未发现支持证明全局无告警的完整快照或有效性标志。因此本轮不发明全局正常判断：

   | 已有证据 | 当前呈现 |
   | --- | --- |
   | 断线，无保留的未恢复告警 | 告警状态待确认：连接已断开。 |
   | 已连接，但没有完整有效依据 | 告警状态待确认：尚无完整有效依据。 |
   | 所有已收到的告警都有有效、当前连接且未过期的显式恢复/非活动标志 | 已收告警已确认恢复；整体状态待确认。 |
   | 曾收到未恢复告警，但连接断开、记录无效、过期或跨连接 | 仍显示原严重等级、内容及已读入口，标明“状态待确认（保留告警）”；顶部横幅同步注明保留告警。 |

   显式恢复只确认对应的已收记录，不等同于全系统没有活动告警，更不等同于设备正常。旧恢复证据过期后不再宣称当前已确认恢复，也不把曾经明确恢复的告警仅因断线重新作为未恢复故障。空增量、连接成功及本页已读均不能清除活动告警；显式 `valid:false` 的恢复消息不用于消除告警。

2. **本地派生的磁盘未挂载/巡迹 LOST、CONFLICT_2D 告警在连接重建时会消失。** 原本仅依赖当前连接记录，重建连接会使依据失效。现增加最多两个条目的呈现层保留集合，使用原有连接范围、有效性和新鲜度判断；只有新收到的有效当前源状态明确解除原条件，才撤下对应告警。没有增加定时器，没有更改连接重置、状态接收或控制门禁。

3. **实际截图发现涡流区收到样本却没有绘制痕迹。** 新增像素断言在修复前失败。原逐像素 min/max 绘制在单样本桶或常量桶中形成零长度线段，默认平头不显示。仅在原绘制路径设置 `lineCap="round"`，使这些采样点可见；原数据、坐标、独立纵向缩放、保峰分桶、线宽、颜色及刷新周期均不变。风险检查：这一属性不增加绘制路径、数据计算或订阅，不生成插值/假数据；不是改变图表布局或新增装饰。

| 本轮文件 | 修改范围 |
| --- | --- |
| `apps/web_console/console.js` | 仅告警呈现/两条本地告警保留记录，以及涡流 Canvas 线端属性。 |
| `tests/web_console/test_ui_v1_alerts.py`（新增） | 复用 Firefox 和隔离网格图服务，覆盖告警证据、断线/手动及自动重连、长文案、三路覆盖层与全屏、滚动访问、常量涡流复现和固定视口前后对照。 |
| `tests/web_console/test_realtime_refinement.py` | 现有 `_serve_geometry_fixture` 增加可选测试配置回调，默认行为不变；注入接口只存在于测试进程。 |
| `docs/validation/20261030_ui_redesign.md` | 追加本节及证据链接。 |

`index.html`、`theme.css`、`theme.js`、`console_core.js`、`replay.js`、`camera_tuning.js` 与本轮开始时逐字节一致。对 `console.js` 剔除上述两个限定修改块后也与基线逐字节一致，证明没有改通信、订阅、解码、覆盖层坐标、停止操作、录制状态机或刷新逻辑。SHA-256 与范围校验见 [source-scope.json](../../.runtime/s100p/validation/ui_v1/source-scope.json)。未新增应用依赖，保留原有未提交修改；未提交、未推送。

### 固定视口、数据来源和截图

本轮使用 Firefox 158.0/Linux 无头浏览器、独立 profile、缩放 100%、DPR=1；补偿浏览器外框后断言真实内容视口为 1366×768 / 1920×1080。截图 `full:false`，PNG 头尺寸与实际视口一致。前后断线对比均为本地静态服务、无 WebSocket 数据、手动断开、模式未知、零图像帧、滚动 `(0,0)`。测量的全部区域边界及字号 **前后相同**，只改变告警文字，详见 [v1-comparison.json](../../.runtime/s100p/validation/ui_v1/v1-comparison.json)。

| 断线、无数据，同视口同状态 | UI V1 修改前 | 本轮修改后 |
| --- | --- | --- |
| 1366×768 | [前](../../.runtime/s100p/validation/ui_v1/before-disconnected-1366x768.png) | [后](../../.runtime/s100p/validation/ui_v1/after-disconnected-1366x768.png) |
| 1920×1080 | [前](../../.runtime/s100p/validation/ui_v1/before-disconnected-1920x1080.png) | [后](../../.runtime/s100p/validation/ui_v1/after-disconnected-1920x1080.png) |

有流测试使用既有隔离 mock 的三路 JPEG 网格图（1920×1200、1280×720、640×480），经真实 WebSocket 接收/解码/Canvas 展示路径传入，同时传入测试巡迹结果和含长连续诊断 ID 的中文告警。画面明确写有 **UI TEST / NOT HARDWARE**，平台模式为 **mock**。测试服务只绑定 `127.0.0.1`，没有 ROS 或设备接入。这是呈现层测试，不能作为硬件或算法验收。

| 有流固定视口证据 | 1366×768 | 1920×1080 |
| --- | --- | --- |
| 三路图像、巡迹、长告警 | [截图](../../.runtime/s100p/validation/ui_v1/live-long-alarm-1366x768.png) | [截图](../../.runtime/s100p/validation/ui_v1/live-long-alarm-1920x1080.png) |
| 滚动后辅助预览与操作 | [截图](../../.runtime/s100p/validation/ui_v1/scrolled-depth_camera-1366.png) | [截图](../../.runtime/s100p/validation/ui_v1/scrolled-depth_camera-1920.png) |
| 滚动后轨迹与涡流区 | [截图](../../.runtime/s100p/validation/ui_v1/scrolled-eddy-1366.png) | [截图](../../.runtime/s100p/validation/ui_v1/scrolled-eddy-1920.png) |

**全屏条件必须单独解释：** 从上述两种窗口分别进入三路全屏并退出；当前 Firefox 无头环境的原生全屏实际均为 **1366×768**，退出后恢复原窗口视口。因此文件名 `fullscreen-from-1920x1080-*` 表示进入前的窗口，不表示全屏达到 1920×1080。见 [从小窗口进入主相机全屏](../../.runtime/s100p/validation/ui_v1/fullscreen-from-1366x768-mindvision.png)、[从大窗口进入主相机全屏](../../.runtime/s100p/validation/ui_v1/fullscreen-from-1920x1080-mindvision.png)。三相机的实际全屏尺寸及几何检查完整记录在 [live-layout.json](../../.runtime/s100p/validation/ui_v1/live-layout.json)。

涡流问题另做确定性对照：暂停隔离推流，每次打开新页面，只发送完全相同的两条常量测试样本 `[100,105]`，时间戳一致，单位标注 `UI_TEST_raw_count`，相同视口与滚动目标。原版 Canvas 无绘制痕迹，修改后存在采样点；不是用不同数据制造前后效果。详见 [constant-eddy.json](../../.runtime/s100p/validation/ui_v1/constant-eddy.json)。

| 确定性常量样本 | 修改前 | 修改后 |
| --- | --- | --- |
| 1366×768 | [前](../../.runtime/s100p/validation/ui_v1/before-constant-eddy-1366x768.png) | [后](../../.runtime/s100p/validation/ui_v1/after-constant-eddy-1366x768.png) |
| 1920×1080 | [前](../../.runtime/s100p/validation/ui_v1/before-constant-eddy-1920x1080.png) | [后](../../.runtime/s100p/validation/ui_v1/after-constant-eddy-1920x1080.png) |

### 本轮实际测试结果

- 新增浏览器检查 **4 项通过**，既有浏览器主要回归 **3 项通过**，合计 **7 passed, 1 deselected（56.30s）**；排除项为可选持续运行 soak，本轮没有重新执行它。
- 既有 `test_console_core.js` **8 项通过**，包含独立新鲜度、断线/错误区分、精确帧关联、控制能力门禁、数据语义与脱敏。
- 手动断线、手动重连、服务端关闭 WebSocket 后自动重连均验证保留未恢复告警；空增量不能清除告警，无效恢复被拒绝，明确有效恢复只作用于对应来源，过期恢复证据退回待确认。
- 两种视口下三路原始图像/overlay 像素尺寸一致、CSS 边界一致、宽高比误差小于 0.005、图像位于容器内、精确帧匹配为 `matched`；检查源坐标中线像素为不透明绿色（允许 Canvas 合成的 RGB 1 级量化误差）、图像角落无误绘。全屏及退出后重复验证。
- 长告警在侧栏完整换行、无水平溢出，顶部保留原有摘要长度；动态修改文案后能更新，图像帧计数及位姿继续变化。小屏允许正常向下滚动，辅助相机叠加开关经真实 WebDriver 点击验证；辅助预览、轨迹和涡流画布通过命中检查，未被固定工具栏遮挡。
- 其余四页沿用已有五页/六尺寸浏览器回归，含录制/留证/回放、禁用状态、异常与恢复；相关操作只作用于隔离服务。所有隔离服务断言 `chassis_requests=0`，未向真实机器人发送运动指令，未启动生产硬件脚本。
- `git diff --check` 通过。前两轮截图没有覆盖，旧测试输出本次重定向至 `ui_v1/regression/`。

复现新增检查（使用环境已有测试解释器/依赖，不安装应用依赖）：

```bash
PYTHONPATH=packages/inspection_runtime/src \
UI_V1_BASELINE_DIR=/tmp/ui-v1-baseline \
/tmp/industrial-ui-testenv/bin/python -m pytest -q -s \
  tests/web_console/test_ui_v1_alerts.py
```

最终联合回归用同一解释器调用 `pytest.main` 运行该文件和 `test_visual_console.py -k 'not soak'`，调用前将 `test_visual_console.OUT` 指向 `.runtime/s100p/validation/ui_v1/regression`，保护旧截图。JS 测试使用 VS Code 已附带的 Node 执行 `--test tests/web_console/test_console_core.js`，未安装新依赖。没有 `/tmp/ui-v1-baseline` 时仍可运行当前版本检查，但不会生成历史对照；持久证据已保存在项目 `.runtime/s100p/validation/ui_v1/`。

### 未验证及剩余限制

- 当前接口没有可证明“全系统确认无活动告警”的完整有效依据，故不显示这种结论。这是明确保留的未知状态，不以连接成功或清空列表替代。
- 未执行真实硬件图像/巡迹/告警验收、真实 MCAP 回放、运动操作或长时间硬件运行；未复测 Chromium、Windows、高 DPI 和现场实际 1920×1080 显示器的原生全屏。
- 本轮未修改或处理 D405 USB 停流。继承上一轮硬件限制。
- 已测场景没有剩余失败；长告警增加页面高度属于正常内容流，未为追求一屏而压缩视频、缩小文字、裁切内容或重排 V1 结构。

## 第四轮：视觉巡迹实验操作接入（2026-10-30）

按用户确认在现有巡迹侧栏加入“启动视觉巡迹、停止视觉巡迹、保存实验样本”，保留 V1 主布局。实际后端接入、原始样本索引、固定视口截图、构建和测试结果见 [视觉巡迹接入报告](20261030_vision_controls.md)。连续数据沿用原采集功能；不控制机器人、不新增网页训练。本轮硬件和焊缝精度未验收。

## 第五轮索引：D405 专用实验采集（2026-10-30）

本轮按用户要求新增 D405 专用启动/录制配置，UI V1 布局、样式和交互代码未变。复用原采集与视觉巡迹入口，保留三相机方案。配置清单、隔离 ROS 实际 MCAP 验证、两种固定视口控件回归截图与剩余硬件限制见 [视觉巡迹报告追加章节](20261030_vision_controls.md#追加d405-专用实验采集方案2026-10-30)。
