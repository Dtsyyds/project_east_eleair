# 配置入口和覆盖规则

1. `deploy/env.sh [s100p|x86|rk3588]` 选择构建/运行平台；包选择来自 `platforms/<platform>.json`，视觉算法平台参数另在 `vision/platforms/`，职责不同。
2. 默认系统运行配置 `communication/system.yaml`。`INSPECTION_CONFIG_ROOT` 可指向另一套完整配置树；`INSPECTION_SYSTEM_CONFIG` 或 `deploy/run.sh --config FILE` 明确替换系统 YAML；后者优先。没有隐含深合并。
   `communication/cameras.yaml` 是独立三相机配置，显式启用时要求六路图像；网页显示“三相机采集”，每个新批次保存方案、必需话题和实际硬件配置。默认整机要求不会因某设备缺失而自动放宽。
   `communication/d405_experiment.yaml` 是 D405 专用完整配置，由 `bash system_websocket/run_d405.sh` 加载；仅启动 D405，要求其彩色/深度/双红外四路图像。`record_topics` 是板端显式录制清单（14 个 D405 原始流/标定/内部外参/巡迹话题），不会追加其他设备或 EMAT 数据；网页请求无法替换清单。不配置此字段的旧方案保持原录制范围。每批 `run.json` 记录选定清单；话题被列出不代表已收到消息。
3. `FIELD_DATA_ROOT` 显式覆盖 system.yaml 的 data_root；其默认 `/media/mcap_data` 是外部数据盘，不是旧代码路径。ROS launch 参数/节点参数的显式值覆盖其包内默认配置，保留 ROS 原行为。
4. `collection/data_handoff_profiles.yaml` 为交付入口；`vision/`、`d405/` 分别保留现有算法默认值；视觉配置中历史 `configs/foo.json` 按当前配置文件所在目录解析 foo.json，配置字节和封存哈希保持不变。
5. 各 ROS 包的原生配置仍在 `ros2_ws/src/**/<package>/config`，随该包安装到 share；完整目录索引见 `ros_config_catalog.json`。这些目录是对应配置的唯一源码，不在本目录复制第二套。视觉 ROS 专用 defaults 在 `vision/ros`，其语义不同于算法 defaults。

只读资源与生成物：中央视觉资源通过 tank_vision_ros/setup.py 安装到 share/tank_vision_ros/config；修改权威源后重建。不要人工改 install 下资源。
新增运行产物在 `.runtime/<platform>`。采集数据和封存 run 的 frozen_configs 留在外部 data_root；不改写历史 run。
`FIELD_VISION_EVIDENCE_ROOT` 可指向原封存视觉证据的只读数据根（默认新 runtime/analysis）；不会复制大型原 output。
历史复现脚本 `tests/system/*reproduction.py`、frozen_comparison.py 需要显式 `INSPECTION_BASELINE_FIXTURES`，按原相对路径放置历史源码快照；输出写新 runtime/historical_reproduction，必要的旧 final_report.json 也应作为明确的复现输入准备。
P2E0/E1/E3 诊断也使用该变量核验冻结配置 SHA：当前配置只允许 `camera.sdk_root` 从旧固定路径迁移为 `null`，其余字段必须与原冻结配置完全相同；原 SHA 不改写。冻结图像、标注、历史 timing 和现场快照从 `FIELD_VISION_EVIDENCE_ROOT` 只读读取，新诊断结果写入 runtime/analysis；已有非空输出拒绝覆盖。

`camera/root` 与 `camera/physic` 保留 SDK mvdat，不等同于有效相机内参。rough 外参仍在 inspection_bringup/config/rough_extrinsics.yaml，production_valid=false；其三张证据图片缺失。默认模型路径 null，不代表模型后端可用。

正式 MindVision 适配在 CameraInit 前通过厂商头文件声明的 CameraSetDataDirectory 将 SDK 生成物放入 runtime/mindvision；初始化后按真实序列号从中央 camera/{root,physic}/Data 读取原 mvdat，再应用已有显式节点参数。相同序列号有不同内容时拒绝歧义，使用 MINDVISION_PARAMETER_FILE 明确选择。未找到该序列号文件时保留 SDK 初始参数，不伪造标定。该路径修复的设备实际读取/写入行为仍需下一轮验证；厂商演示程序保留原文与 CWD 行为。

## D405 替换设备

`system.yaml`、`cameras.yaml`、`d405_experiment.yaml` 及硬件 launch 默认允许
`260322276866,352122271190` 两个 SDK 序列号。任接一台即可使用，无需改配置；
运行中拔下后接入另一台，驱动仍按完整允许列表选择设备。仅选择 D405 型号。
同时接入两台时选 SDK 枚举到的第一台，保持当前设备直到断开，不同时采集两台。
需要固定某台时，将 `hardware.d405_serial_number` 改为单个序列号。

此功能依赖本工程的 RealSense 驱动，修改后需重建 `realsense2_camera` 和
`inspection_bringup`，并重启已有服务。驱动按允许列表中每个序列号持有进程锁，
防止另一采集进程在换机时抢占设备。内参与内部外参由当前相机读取；USB 掉线或
降速仍需排查线缆和端口，序列号兼容不代表连接稳定性已经通过验收。

## D405 竖直焊缝巡迹

D405 专用方案的 `vision.algorithm_config` 选择 `d405/vertical_seam.yaml`：
面向当前现场较暗、两侧较亮的竖直焊缝。先比较局部条带与左右背景，再在各行候选中
选择位置和宽度连续的路径。保留原 ROI、55% 覆盖率和 5 像素拟合残差门槛，
短缺失行不补成实测点，长遮挡不跨越。边界代表对比度条带，不等于标定后的物理焊缝宽度。

未指定 `vision.algorithm_config` 的方案仍使用 `d405/experimental.yaml`。
启动视觉巡迹时复制所选完整 YAML 到本次 `VISION-*` 会话，运行中修改源配置不会
改变该会话；下一次启动生效。当前竖缝配置显式使用暗条带，明亮焊缝需另行验证配置。
二维有效不意味着深度连续性、三维定位或机器人控制已通过。

## 工业相机在线调参

上位机工业相机窗口的“相机调参”可读取设备实际值，调整曝光模式、曝光时间、模拟增益、自动曝光目标、白平衡、RGB 增益、饱和度、Gamma 和对比度。“应用预览”只改变当前设备；“保存并沿用”核对回读后原子写入 `camera/tuning/<真实序列号>.json`。界面曝光单位为 ms，文件和 SDK 单位为 μs。范围、增益步长和自动白平衡支持情况来自设备能力。

优先级：SDK 初始值 → 原序列号 mvdat → 驱动 YAML / launch 或视觉任务显式参数 → 序列号调参 JSON。ROS 工业相机驱动和 `VendorCameraSource` 均在启动采集前加载该 JSON，因此保存后重启上位机或启动后续视觉任务会沿用。分辨率、发布频率、编码及几何标定仍由对应任务配置管理，不写入调参 JSON。已经运行的独立视觉进程需重启才能加载新配置。

当前相机 `046041920527` 的配置保存于 `camera/tuning/046041920527.json`。2026-10-29 实测：手动曝光约 70877 μs、模拟增益 4×、AE 目标 70、白平衡 off、RGB 131/110/147、饱和度/Gamma/对比度均 100。当前设备不支持自动白平衡。三相机 launch 使用 1920×1200、10 Hz 发布；包内默认分辨率不等于当前 launch 的实际输出。

采集、预检或回放期间禁止修改参数，仍可读取。请求按序列号及实际参数版本校验，避免多页面覆盖旧值；SDK 设置或保存失败会尝试回滚，回滚失败时停止输出并报告故障。新采集批次 `run.json.camera_settings` 记录启动时设备实际参数，包括尚未保存的预览参数。完整实测见 `docs/validation/20261029_camera_tuning.md`。
