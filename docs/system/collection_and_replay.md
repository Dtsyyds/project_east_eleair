# 采集、停止、回放、RRD 硬件短测

以下为用户下一步命令，本任务没有执行真实采集。MCAP 已由本地依赖恢复，先确保 preflight 成功；确认 SSD 可用、没有其他相机 owner、串口/网络/设备配置与真实标定状态。保留原默认配置，以新 run_id 和新分析目录运行，不能覆盖历史 run。

```bash
cd /home/sunrise/east_eleair
S="$PWD/tank_inspection_system"
source "$S/scripts/env.sh"
bash "$S/scripts/preflight.sh"
RUN_ID="REFACTOR-HW-$(date +%Y%m%d_%H%M%S)"
export FIELD_RUN_ID="$RUN_ID"
bash "$S/scripts/collect.sh" run_id:="$RUN_ID"
# 采集60秒后在另一终端 source同一env，再停止。默认 tracker=false。
bash "$S/scripts/stop.sh" "$RUN_ID"
A="/media/mcap_data/analysis/$RUN_ID/refactor_hw_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$A/configs" "$A/metrics" "$A/outputs" "$A/logs"
bash "$S/scripts/validate.sh" "/media/mcap_data/runs/$RUN_ID" --report-dir "$A/metrics"
export FIELD_ANALYSIS_DIR="$A/replay"
bash "$S/scripts/replay.sh" "$RUN_ID" sensor-only 1.0 0 60
# 独立 subscriber 记录类型/QoS/header/原速率数量；publisher report不能代替接收验收。
"$FIELD_RRD_PYTHON" -m inspection_rerun.run_rrd_builder   --run-dir "/media/mcap_data/runs/$RUN_ID"   --config "$VISION_ROOT/ros2_ws/src/inspection_rerun/config/rerun.yaml"   --output "$A/outputs/$RUN_ID.rrd" --mode strict --camera-mode preview
bash "$S/scripts/handoff.sh" --run-dir "/media/mcap_data/runs/$RUN_ID" --profile front_camera   --output-root "/media/mcap_data/deliveries/$RUN_ID" --destination "/media/mcap_data/deliveries/$RUN_ID/front_camera" --materialize --copy-method python
bash "$S/scripts/handoff.sh" --run-dir "/media/mcap_data/runs/$RUN_ID" --profile front_camera   --output-root "/media/mcap_data/deliveries/$RUN_ID" --destination "/media/mcap_data/deliveries/$RUN_ID/front_camera" --verify-only
```

RRD 交付命令（使用已经存在的新分析目录）：

```bash
bash "$S/scripts/handoff.sh" --run-dir "/media/mcap_data/runs/$RUN_ID" --profile rerun_viewer \
  --rrd-manifest "$A/outputs/rrd_manifest.json" --output-root "/media/mcap_data/deliveries/$RUN_ID" \
  --destination "/media/mcap_data/deliveries/$RUN_ID/rerun_viewer" --materialize --copy-method python
```

核查报告：run 的 complete/sha256/停止过程，analysis metrics 的动态有效header/数量/间隔与video严格映射，独立subscriber的ROS报告，RRD `.manifest.json`/SHA/verify日志（以 builder 输出记录为准），delivery manifest/SHA。6Hz真实硬件门槛、SSD保护与异常断开、同步开启时FusionIndex需现场复测。同步默认采集关闭，不擅自更改运动/EMAT等。

algorithm-replay 保留原入口，可在另一个新分析目录使用相同速率；算法状态必须独立验收。不能以降速后结果冒充原速率。


当前媒体目录不可见时，先由用户恢复原数据卷/只读历史目录可见性。复核命令会写新报告，保留初始清单：

```bash
python3 "$S/scripts/check_source_preservation.py" --update-closeout
```

快照/ROI/provenance 在需要的独立短测中显式启用 `enable_tracker:=true`；默认短测 tracker=false 时服务不会存在。实际手动快照服务为 `/stable_seam_node/save_snapshot`，类型 `std_srvs/srv/Trigger`。ROI 原录制参数是 `record_front_raw_burst:=true raw_burst_mode:=roi raw_burst_duration_sec:=10`；provenance 使用原开关与容量，不增加二次算法常驻计算。

## 本次复核与媒体恢复

```bash
cd /home/sunrise/east_eleair
S="$PWD/tank_inspection_system"
source "$S/scripts/env.sh"
bash "$S/scripts/verify_offline.sh"
python3 "$S/scripts/check_installation.py" --update-closeout
python3 "$S/scripts/check_static.py" --update-closeout
python3 "$S/scripts/check_source_preservation.py" --update-closeout
bash "$S/scripts/preflight.sh"
```

默认验证日志追加到现有 `reports/refactor_v1/verify_metadata_workflow.txt`，每组命令、退出码与结果写 `final_report.json.closeout.checks`；临时 fixture 自动清理。最后两条目前返回 2，原因是历史媒体不可见，不是 MCAP 失败。preflight 的注册检查本身不能代替实际 MCAP roundtrip；实际证据来自 video_consumers 与 rrd_including_mcap。

当前 `findmnt -T /media` 指向根文件系统 `/dev/mmcblk0p17`；`lsblk` 只列出本机 eMMC，没有外接 SSD，也没有媒体 bind mount。目录不存在，不能通过修改权限或另建假 runs 修复；不要将系统 eMMC 当成数据 SSD。进程 1 的 mount namespace 无权限检查，是否被宿主机隔离尚不能确认。由宿主机操作员重新接入原 SSD，执行以下识别步骤：

```bash
lsblk -o NAME,PATH,TRAN,FSTYPE,SIZE,MOUNTPOINTS
findmnt -T /media
sudo blkid
# 接入后查明原数据分区的真实设备路径/UUID；本次未发现该设备，不能填写 /dev/sdX 猜测。
# 若已经挂载在其他目录，只设置真实目录，不改动源数据：
# export FIELD_DATA_ROOT=/实际挂载目录
# 若确认为未挂载 ext4 SSD，由操作员只读挂载（不格式化、不改 fstab、不强制 remount）：
# sudo mkdir -p /media/mcap_data
# sudo mount -o ro,noload /已确认的SSD分区 /media/mcap_data
# 若宿主机能看到而执行环境看不到，将同一实际目录只读绑定给当前执行环境。
```

可见后优先只读复核 `FIELD-VEHICLE-20260928_115534` 的原封存清单/库存/实际 SHA，报告写独立分析目录；不存在则列出实际已封存 run 并记录选择依据。不能将 stat 一致视为实际 SHA 验证，不能更改旧报告。只读可见即可审计；新采集另需真实可写数据卷及 SSD 保护检查。完整源数据审计、硬件接口与 60 秒短测通过后才执行 30 分钟稳定性；本阶段 hardware_revalidated=false、algorithm_field_validation_pending=true、production_ready=false。

## 无线 WebSocket 与唯一配置

唯一配置为 `physic_bev/src/system_websocket/agv_bridge/config/tank_inspection.yaml`；install 中是构建复制品，不维护第二份手工配置。修改源配置后执行 `build.sh --wireless-only`。launch 未显式传入 URI/端口时沿用配置值（参数哨兵 __config__），不会用另一份默认值覆盖配置。启动方式：

```bash
bash "$S/scripts/build.sh" --wireless-only
source "$S/scripts/env.sh"
ros2 launch agv_bridge tank_inspection.launch.py
# 可选：无线 LAN 上的已有控制服务器，无公网要求
# ros2 launch agv_bridge tank_inspection.launch.py server_uri:=ws://实际局域网服务器:9100 local_server_port:=9001
# 独立安装的注册测试（不启动驱动）
colcon --log-base "$S/reports/refactor_v1/wireless_preview_build_logs" test \
  --build-base "$S/build_wireless_preview" --install-base "$S/install_wireless_preview" --packages-select agv_bridge
colcon test-result --test-result-base "$S/build_wireless_preview" --verbose
```

默认本地服务 `ws://板端局域网IP:9001`，远端 URI 为空；机器人 ID `tank_inspection_robot`。沿用 `header.robot_id/msg_type + payload` 协议。启动立即注册订阅并每 200ms 回报自检；六设备为 `fisheye/mindvision/depth_camera/eddy_current/emat/chassis`，另列 lidar/lidar_pose。动态 header 非正、重复、回退，非法布局或过期均失败；未配置/未收到消息是 unverified。底盘 headerless 心跳或 Pose2D 标注 host_monotonic_receipt。自检证据仅为近期有效 ROS 数据，不能替代串口协议、设备身份和硬件故障验收。

默认已知接口为工业相机 `/camera/front/image_raw`、D405 RGB `/d405/camera/color/image_raw`、涡流 `/eddy/data` 与 `/eddy/channel_mean`、雷达 `/livox/lidar`、位姿 `/aft_mapped_to_init`。D405 视频按用户要求改为 RGB；原 Depth ROS topic 保留，采集生产参数不变，RGB 必须由唯一相机 owner 显式 enable_color。鱼眼、EMAT、底盘状态/命令 topic 默认空，必须按真实驱动确认后配置。EMAT 消息要求 `inspection_interfaces/msg/UltrasoundFrame` 的实际通道/样本/采样率；没有驱动不得填造。涡流原始波形与均值分别验证，均值不伪装波形。底盘支持 String online/connected/ok 或 Pose2D 活性，以及 Twist 或 Float64MultiArray `[vx,vy,wz]` 命令；必须确认原驱动单位 m/s、rad/s 和类型后启用。旧 `/agv_pose`、`/cmd_vel` 只是其他机器人中的线索，不自动套给当前底盘。

客户端发送：

```json
{"header":{"robot_id":"tank_inspection_robot","msg_type":"check"},"payload":{}}
{"header":{"robot_id":"tank_inspection_robot","msg_type":"command"},"payload":{"camera":{"command":"stop_stream"}}}
{"header":{"robot_id":"tank_inspection_robot","msg_type":"command"},"payload":{"camera":{"command":"start_stream"}}}
{"header":{"robot_id":"tank_inspection_robot","msg_type":"command"},"payload":{"agv":{"command":"move","parameters":{"vx":0.1,"vy":0.0,"wz":0.0}}}}
{"header":{"robot_id":"tank_inspection_robot","msg_type":"command"},"payload":{"agv":{"command":"stop"}}}
```

`response` 是受理/ROS 发布结果；移动与停止返回 execution=published_not_hardware_acknowledged，不是物理执行 ACK。默认 control_enabled=false；显式启用要求命令 topic、有效底盘消息，速度上限默认 0.2m/s 与 0.5rad/s，0.5 秒未续发或心跳过期即发布零速度。配置 control_token 时 payload 需携带匹配 token。最终硬件停机机制仍须按真实底盘复测，不能将软件 watchdog 宣称物理闭环。

双视频在 `status_sensor.payload.video`：camera_id 为 fisheye/depth_camera，encoding=jpeg_base64，data 为 JPEG，source_encoding、width/height、原 frame_id 和 stamp 保留。每流仅保留最新 ROS 图像；当前罐体配置目标15 FPS、宽640，video_fps允许大于0且不超过30，report_period_ms=20按deadline调度，自检仍约200ms回报。preview_dropped显式累计被覆盖帧，不用于完整录制。目标频率不是实测保证值；页面分别显示实际解码与收到FPS，消息另含preview_target_fps/encode_wall_ms。16UC1 深度仅作原始单位灰度预览，source_depth_unit=unavailable、visualization_only=true，不能作为已标定深度。工业相机只参与自检，不额外复制全图。未连接控制端不编码预览。

`status_update.payload.lidar` 提供 x/y/z（m）、完整四元数、angle（归一化四元数计算 yaw，rad）、frame_id/child_frame_id，asset_coordinate_valid=false。只消费已有 Odometry，不自动启动 FAST-LIVO2 或 mapping。精确纳秒 `stamp_ns` 为十进制字符串，另提供 `stamp_sec/stamp_nanosec` 数字，避免 JavaScript 浮点精度损失；原 MCAP/sidecar 格式不变。无有效位姿时不伪造坐标。

本地 offline 归档在构建前验 SHA，运行与测试不下载。新增依赖归档与已有本机 Ubuntu/Humble/设备 SDK 一起构成离线运行环境；未知鱼眼/EMAT/底盘驱动未提供，因此其设备 SDK 和现场控制接通仍是待配置项。仅使用 HTTP 代理指向不可用地址的本地 WebSocket/ROS fixture 来验证无公网依赖，不等同物理 WLAN 测试。退出 SIGINT 保留有限期关闭握手；端口占用启动失败返回 2，不停止已有监听者。

## 回放本次行为修复

回放所有原位置参数、rate=1.0/0.5、offset、duration、QoS、消息 header 和原数据格式保留。启动现在先做最多 2 秒 DDS 发现与短暂连接稳定等待，然后开始原媒体时间轴，避免首条 CameraInfo/视频在 reader 尚未就绪时丢失。`--discovery-wait-sec` 是新增参数；设 0 可恢复立即开始。包装脚本只在显式设置 FIELD_REPLAY_DISCOVERY_WAIT_SEC 时透传此参数；默认值只在原 Python 入口定义。发现结果写 startup_discovery，仅是 endpoint 证据，transport_acceptance 仍需独立接收验证。没有 subscriber 时有界退出等待并继续原发布流程，不能宣称收到数据。

```bash
# 需要原立即开始行为时（无首帧交付保证），仍使用原入口：
FIELD_REPLAY_DISCOVERY_WAIT_SEC=0 bash "$S/scripts/replay.sh" "$RUN_ID" sensor-only 1.0 0 60
```

MID360 原 `/livox/imu_raw` 的加速度是 g；原回放曾只改名为 `/livox/imu`，数值仍是 g，违反该 topic 的 m/s² 契约。本次复用已经安装的 `inspection_livox_adapter/livox_imu_adapter_node`，通过两条含 PID 的内部 topic 转换后按原 public topic、reliable/volatile/depth10 发布。没有复制转换公式，没有启动 Livox 驱动。标准重力、gyro 和未知 orientation 的处理均沿用现有 C++实现，header/stamp/frame_id 不变；原 MCAP 保持 g 和原 SHA。

仅选中 native IMU 的 sensor-only/full-sensor 需要该已安装依赖；相机-only/algorithm-replay 不启动它。适配输出会实际核对数量/header，积压最多 200，不能静默丢弃离线结果；未完成转换明确失败。关闭只停止本次拥有的被动适配进程，有期限，不停止现场其他进程。源同时含可选中的 raw/SI 且会造成同一 output owner 歧义时明确拒绝。当前本机该包与二进制已存在，离线可用，不另复制 ROS 包或升级它。

新增无线预览是 latest 策略，不作无损录制。80 帧/每路、20Hz fixture 的接收/预览/覆盖/未接收数量单独记在 closeout.checks.wireless.preview_load；出现的 DDS 未接收数量保留，不以降速消除。invalid 与 accepted 分别计数，非法帧不伪装成被覆盖的有效帧。主采集/完整离线处理仍执行原完整性契约。

## 上位机 WebSocket 验证页面

页面路径为 `tank_inspection_system/websocket_console/index.html`，与 `console.js` 一起复制即可在上位机浏览器离线打开。无需安装 Python、npm 或前端框架；不依赖公网服务。已实测浏览器是 Firefox 140.0.4。上位机与板端需通过实际局域网相互可达。

板端使用原被动桥接入口；此 launch 不启动相机、mapping、底盘或采集：

```bash
cd /home/sunrise/east_eleair
source tank_inspection_system/scripts/env.sh
ros2 launch agv_bridge tank_inspection.launch.py
```

在上位机双击 HTML，填写 `ws://板端实际IP:9001`，点击“连接”。页面只自动发送一次 check，状态与视频来自板端现有周期回报。无真实消息时显示未验证/失败，不生成模拟成功数据。需要通过板端提供页面时，在另一终端运行以下静态服务，然后由上位机访问 `http://板端实际IP:8080/`；默认 WebSocket 地址取该主机的 9001 端口：

```bash
cd /home/sunrise/east_eleair
python3 -m http.server 8080 --bind 0.0.0.0 \
  --directory tank_inspection_system/websocket_console
```

验证操作顺序：连接后检查六设备状态及 topic/type/header；分别核对鱼眼、D405 视频的源 frame_id、精确 stamp_ns 和板端 latest 覆盖数量；核对雷达位姿的米、弧度、四元数及坐标系；点击停止/开启预览，检查 response 与视频更新；导出 JSON 留存结果。关闭连接会取消自动重连，设备显示和位姿不会继续保持有效。无效 JSON、二进制、超大回包、其他 robot_id 和不可解码 JPEG 分别记录。图像只保存最新预览，日志最多 200 条，嵌套元数据有上限，导出不包含 JPEG 正文或 Token。

涡流设备的 eddy_current 是 raw/mean 的聚合活性，不能据它推定原始波形有效；页面分别显示实际 eddy_raw 与 eddy_mean。深度 JPEG 仅用于显示，不能还原 uint16 深度或用于测量。精确纳秒保持十进制字符串；“距收到”使用浏览器接收时间，不冒充端到端延迟或传感器时间差。

底盘命令区域默认折叠且速度发送关闭。人工启用本页面命令、填写配置要求的 Token 后，才能发送一次 `agv.move`；页面测试上限为各线速度 0.2m/s、角速度 0.5rad/s，板端仍执行自己的验收。停止按钮发送 `agv.stop`。仅在本页面命令已人工启用的连接中，用户断开会尝试发送一次 stop；断线本身不能保证送达，仍依赖已有板端 watchdog。Token 不持久化且不进入日志/导出；传输协议沿用原实现。回包没有 request_id，并向连接客户端广播，页面明确显示其共享语义，不认定为独立硬件 ACK。

纯前端不需要编译，不改 ROS 安装或配置。复测真实浏览器与安装版桥接（临时 fixture，隔离 domain，不启动硬件）：

```bash
cd /home/sunrise/east_eleair
source tank_inspection_system/scripts/env.sh
TANK_CONSOLE_FIXTURE_DIR="$(mktemp -d /tmp/tank-console-test-XXXXXX)"
ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  HTTP_PROXY=http://127.0.0.1:9 HTTPS_PROXY=http://127.0.0.1:9 ALL_PROXY=http://127.0.0.1:9 \
  python3 -m pytest -q -s -p no:cacheprovider \
  tank_inspection_system/tests/test_websocket_console.py \
  --basetemp="$TANK_CONSOLE_FIXTURE_DIR"
# 核对结果后，仅清理上面 mktemp 创建的本次 fixture 目录。
```

测试使用本机已有 Firefox 内置 [Marionette](https://firefox-source-docs.mozilla.org/remote/marionette/) 与现有 ROS/pytest/websockets，不下载浏览器、不引入 Selenium/geckodriver；Firefox 缺失时明确 skip，不能称浏览器验收通过。记录集中在 `final_report.json.closeout.websocket_console`，日志复用 `verify_metadata_workflow.txt`。实际设备/WLAN/SSD 与 D405 融合算法验收仍保留未验证状态。

## D405 单设备实机验证（2026-10-04）

用户接入 D405 后已实际启动该相机与被动桥接、静态页面；没有启动其他传感器、mapping、底盘或正式采集。实测结果集中在 `final_report.json.closeout.d405_hardware_validation`，原软件自检及整体硬件未验收状态保留。设备真正的 SDK 序列号为 `352122271190`；USB 描述符序列号 `254723075213` 是 firmware_update_id，不能拿它作为驱动 serial_no。USB 为 5000 Mbps / SDK USB 3.2，实际驱动与 SDK 均使用已安装版本。

Depth、Infra1、Infra2 的现有 topic 均在 `/d405/camera/` 下，图像分别是 `depth/image_rect_raw`（16UC1）、`infra1/image_rect_raw`、`infra2/image_rect_raw`（mono8），各有 `camera_info`、`metadata`。三个图像的单一发布者为 `/d405/camera`。ROS 驱动 `base_realsense_node.cpp::fix_depth_scale` 将 16UC1 标准化为毫米，即 ROS 数值乘 0.001 才是米；不要再次乘设备原生 depth_units。原生 SDK 深度比例本次未独立读取。Factory CameraInfo 有实际 K/P；Infra2 CameraInfo 按驱动的右目投影约定使用左目 optical frame 和非零 P[3]，与 Infra2 图像 frame_id 不同，不能直接当成同一个坐标系。Depth→Infra1 外参为单位变换，Depth→Infra2 平移约 -0.01804 m；D405→车体现场外参没有验证。

独立 25 秒测量接收 Depth 738、Infra1 748、Infra2 740 帧，帧率分别约 29.52/29.91/29.60，零、重复、回退 header 均为 0，超过 50 ms 的间隔分别有 12/2/10 个。并行 Firefox/回归测试下曾降至约 26.0/29.0/27.5 FPS，两组结果均保留，不将观测缺口归零或推断为无损。metadata 的 clock_domain 是 hardware_clock，经原驱动映射到 ROS 时间，尚未验证外部传感器硬同步。

实际 WebSocket 收到 64 条深度 JPEG，640×362，约 2.54 FPS；上限 5 Hz 不是实测保证值。62 条源时间戳可在独立 ROS 接收集合中找到，其余 2 条没有独立接收证据；不填补或改写时间戳。Firefox 实际解码 5/5 条，无解码错误，底盘命令未启用。预览保留原始时间戳，使用 latest 策略且不能还原深度用于测量。现场画面存在无效深度，当前摆放场景/工作距离和焊缝精度均未验收。鱼眼预览保持未验证，不能用 Infra1 冒充鱼眼。

相机收到 SIGINT 后约 1.50 秒正常关闭并释放设备，随后以相同配置重启。重启后以有界临时测试录制 9 条图像与 3 条 CameraInfo，MCAP 写入、关闭、读取、类型/CDR、精确 header、序列化 SHA 均一致。测试数据归入报告后清理；不写 SSD 历史 run。MCAP 回归测试 1 项通过，97 个源/安装文件 SHA 一致，13 个原 CLI/launch 检查通过。

当前测试服务可直接访问：`http://192.168.127.10:8080/` 或 `http://192.168.1.6:8080/`，点击连接。页面自动使用对应主机的 `ws://...:9001`。本次浏览器 E2E 在板端 loopback 完成，不能据此宣称上位机无线网络已实测。

已确认两个缺失 Humble/aarch64 运行库并放入 `offline/debs`，构建入口会核验 SHA 并解压到 `offline/sysroot`。无需联网重新下载；没有 apt 安装/SDK 升级。`env.sh` 增加已存在的 SDK 2.58 库路径，匹配驱动编译版本；该 SDK 构建目录承担运行用途，必须保留。

需要停止本次测试服务时，以下命令只操作本次登记且 PID 启动时间仍匹配的进程组；不停止其他采集或驱动：

```bash
python3 - <<'PY'
import json, os, signal
from pathlib import Path
session = Path('/tmp/tank_d405_session_path').read_text().strip()
for item in json.loads((Path(session) / 'session.json').read_text())['processes'].values():
    stat = Path('/proc') / str(item['pid']) / 'stat'
    if stat.exists() and stat.read_text().rsplit(')', 1)[1].split()[19] == item['start_ticks']:
        os.killpg(item['process_group'], signal.SIGINT)
PY
```

停止后，在一个终端按 RGB 预览配置重新启动唯一 D405 owner（保留 Depth/Infra1/Infra2，确认没有另一个 camera 节点/进程正在使用设备）：

```bash
cd /home/sunrise/east_eleair
source tank_inspection_system/scripts/env.sh
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
D405_CHECK_CONFIG="$(mktemp /tmp/d405-check-XXXXXX.yaml)"
printf '%s\n' 'depth_module.global_time_enabled: true' \
  'depth_module.error_polling_enabled: false' > "$D405_CHECK_CONFIG"
ros2 launch realsense2_camera rs_launch.py camera_namespace:=d405 camera_name:=camera \
  serial_no:=_352122271190 device_type:=d405 enable_color:=true \
  enable_depth:=true enable_infra1:=true enable_infra2:=true \
  depth_module.depth_profile:=848,480,30 depth_module.infra_profile:=848,480,30 \
  depth_module.color_profile:=848,480,30 \
  config_file:="$D405_CHECK_CONFIG" diagnostics_period:=0.0 publish_tf:=true \
  wait_for_device_timeout:=10.0
rm -f -- "$D405_CHECK_CONFIG"
```

launch 对两项设备原生选项打印 supported-parameters 警告，但仍将 YAML 传给节点；本次 GetParameters 实际确认二者为 false。`pointcloud__neon_.enable` 和 `align_depth.enable` 也为 false。这里生成的是本次启动参数快照，不修改生产配置。另两个终端使用前文现有 `ros2 launch agv_bridge tank_inspection.launch.py` 和 `python3 -m http.server ...` 即可恢复页面；桥接终端同样设置 ROS_DOMAIN_ID=0、ROS_LOCALHOST_ONLY=0。本阶段只验证 D405 数据通路，SSD、其他硬件、焊缝融合算法、现场标定及持续无丢帧验收仍待执行。

用户随后启动桥接时遇到两项运行环境问题，证据追加在同一 D405 报告字段内。9001 端口被本次保留的桥接占用时，重复 launch 会退出 2；已只停止登记的测试桥接，由用户终端管理当前桥接，不能同时再启动一份。随后 WebSocket 握手正常但 D405 无数据：测试相机的 ROS_LOCALHOST_ONLY=1，而用户桥接为 0。本机对照实测，设 0 的订阅者发现不到该相机，设 1 的订阅者收到 144 帧。已将本次拥有的相机按同一配置重启为 ROS_DOMAIN_ID=0、ROS_LOCALHOST_ONLY=0，与用户桥接一致，未停止用户桥接或修改生产 YAML。修复后 8 秒收到 239 条 ROS 深度消息及 20 张有效 JPEG，自检恢复 verified；远端 192.168.1.23 的 WebSocket 连接可见，但远端浏览器的实际显示仍需操作者确认。

以后先在相机与桥接两个终端中 source 同一 `env.sh`，再设置相同的 ROS_DOMAIN_ID 和 ROS_LOCALHOST_ONLY，然后启动节点。环境变量在进程创建时生效，仅在新终端 export 不会改变已运行节点。离线 fixture 仍使用原隔离 domain 和统一的 localhost=1；运行时 0/1 必须成对一致。ROS_LOCALHOST_ONLY 管的是 ROS DDS 发现/通信，不是 WebSocket 的 LAN 监听地址。若页面连接正常但设备 received 始终为 0，应先查相机发布和两端 ROS 环境，不放宽自检。

用户后续要求只回传 RGB，当前唯一桥接 YAML 的 `tank_inspection.depth_camera_topic` 已改指 Color topic；协议 camera_id=`depth_camera` 保持兼容，`source_encoding=rgb8` 表示实际画面来源。RGB8 在现有桥接中转换为 BGR 后编码 JPEG，页面显示 RGB 画面，不拿深度伪彩冒充 RGB。通用 16UC1 预览实现仍可通过明确配置选用，但不是当前默认视频源。D405 RGB 与深度同步/配准仍需各自的 CameraInfo/外参，视频预览不代表融合或几何验收。

早期RGB桥接保存在 `build_wireless_rgb/install_wireless_rgb`，原 `install_wireless` 也保留。后续显示频率修复使用独立 `build_wireless_preview/install_wireless_preview`；`build.sh --wireless-only` 与 `env.sh` 当前默认选择preview安装。旧终端已有FIELD_WIRELESS_INSTALL时不会自动覆盖，需显式选择下方路径。回退只需将该变量改为原安装、重新source并重启自己管理的桥接，不需要Git reset。自行指定构建位置可设置FIELD_WIRELESS_BUILD、FIELD_WIRELESS_INSTALL、FIELD_WIRELESS_LOG；当前preview安装YAML与唯一源YAML SHA一致。新旧桥接不能同时占用9001。

### RGB 时间戳回绕排障

2026-10-04 曾出现连接正常但零视频：当前 SDK 的 32 位微秒硬件计数回绕，hardware_clock 经驱动固定锚定后退到桥接已接受的最大时间之前。桥接保留回退拒绝与历史错误计数，没有放宽验收。本次预览通过已有 SDK 的 global_time 时间源恢复，5 秒收到并解码13张RGB JPEG。其后同一owner/桥接的约78分钟区间中，global_time、raw硬件计数和frame_number差值支持跨过一次回绕，桥累计新增140325帧全部接受、没有新增回退拒绝；这是区间累计证据，不是逐帧边界捕获。生产hardware_clock默认模式及完整长采集仍未验收；生产采集配置未改。

当前相机已运行时，无需再启动 owner：

```bash
source /home/sunrise/east_eleair/tank_inspection_system/scripts/env.sh
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ros2 param get /d405/camera depth_module.global_time_enabled
# 用户 RGB 预览模式的显式修复；正式采集应冻结配置并另验时钟
ros2 param set /d405/camera depth_module.global_time_enabled true
ros2 param get /d405/camera enable_color
ros2 topic info /d405/camera/color/image_raw --verbose
```

## D405融合实验与后续模块调试

模块接入和排障全流程见 [module_development_and_debugging](module_development_and_debugging.md)；数据抽取、标注、训练/ONNX、三组消融、ROS和加速命令见 [D405框架README](../../d405_seam_tracking/README.md)。相机已接入时只启动被动算法，不再启动owner：

```bash
cd /home/sunrise/east_eleair
source tank_inspection_system/scripts/env.sh
F="$PWD/d405_seam_tracking"
set +u; source "$F/ros2_ws/install/setup.bash"; set -u
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
ros2 launch d405_seam_ros tracking.launch.py config:="$F/configs/experimental.yaml"
# 显式RGB输入：追加stream:=color，深度按factory外参配准
# 有限60秒分析（独立新路径，可写根不必是历史SSD）：
A="/tmp/d405_$(date +%Y%m%d_%H%M%S)"
ros2 run d405_seam_ros d405_seam_node --config "$F/configs/experimental.yaml" \
  --stream infra1 --depth-scale-m .001 --duration 60 --output "$A" --overlay
ros2 topic echo /vision/d405_seam/diagnostics --once
```

最后echo应在另一终端、相同环境运行。不要同时启动上面的无限launch和有限run，两者二选一。最新8秒Infra1/RGB实机链路分别处理235/179对，全部LOST（中央ROI有效深度低、视觉不连续）；未调参消除失败。先将真实目标置于当前配置0.07–0.60米范围并检查ROI有效深度，再判断焊缝证据。有效焊缝精度/完整几何分支实时门槛、60秒/30分钟稳定性仍待适用场景复测。结果和源时间/drop分别登记，不将预览FPS当完整录制FPS。

## 显示帧率修复与复测

3Hz由旧200ms报告轮询、5FPS上限及elapsed计时跳槽共同造成；新罐体配置report_period_ms=20、video_fps=15，deadline保相位，错过时跳过旧槽而不突发积压帧。其他机器人默认轮询仍200ms；设备/transport状态约200ms，运动控制和严格header检查保持。旧配置省略参数仍保留5FPS默认值。浏览器“显示”统计实际JPEG onload成功，“收到”统计WebSocket消息；目标FPS、传感器FPS、算法FPS各自独立。

本机实测原始RGB 2.59→14.94FPS（180/180 JPEG解码）；Color＋Depth融合overlay处理11.98Hz，20.2秒实际Firefox解码11.43FPS，完整回调P95 91.8ms。两秒窗口最低9.77FPS，不能声称严格每窗口保证10Hz；420帧均LOST，不是有效焊缝准确性验收。物理无线、其余传感器并行负载、有效焊缝轨迹及30分钟稳定性仍待实测。实际后端classical_cpu/numpy_cpu，没有BPU加速声明。当前9001保留运行RGB服务，8080页面保留；不要再启动占用9001的第二份桥接。

上位机打开 `http://192.168.1.6:8080/`（另一网口 `http://192.168.127.10:8080/`），Ctrl+F5刷新，再连接 `ws://对应板端IP:9001`。若用复制到上位机的HTML，需要同时更新同目录console.js。

```bash
cd /home/sunrise/east_eleair
export FIELD_WIRELESS_INSTALL="$PWD/tank_inspection_system/install_wireless_preview"
source tank_inspection_system/scripts/env.sh
ros2 pkg prefix agv_bridge  # 应输出install_wireless_preview/agv_bridge
# 仅当9001的原桥接已由其所有者Ctrl-C停止时启动：
# ros2 launch agv_bridge tank_inspection.launch.py
```

查看融合图像时用两终端；都source上述相同环境，D405驱动继续由原唯一owner运行。先有限时测量，输出目录必须不存在：

```bash
F="$PWD/d405_seam_tracking"
set +u; source "$F/ros2_ws/install/setup.bash"; set -u
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
A="/tmp/d405_display_$(date +%Y%m%d_%H%M%S)"
ros2 run d405_seam_ros d405_seam_node --config "$F/configs/experimental.yaml" \
  --stream color --depth-scale-m .001 --overlay --duration 60 --output "$A"
# 第二终端，先source上述环境，再启动只订阅overlay的调试桥：
ros2 run agv_bridge websocket_bridge_node --ros-args \
  --params-file "$PWD/physic_bev/src/system_websocket/agv_bridge/config/tank_inspection.yaml" \
  -p local_server_port:=9002 -p tank_inspection.depth_camera_topic:=/vision/d405_seam/overlay
# 页面改连ws://板端IP:9002，查看实际“显示/收到”FPS。
# 第三终端可只读查看完整回调、overlay耗时和drop：
ros2 topic echo /vision/d405_seam/diagnostics --once
```

60秒结束后检查 `$A/result_manifest.json` 的processed/latest_dropped/sync/actual_processing_fps、callback_window_p95_ms、overlay_window_p95_ms及LOST/有效轨迹比例；JSONL保留每帧业务结果。complete仅表示分析正常关闭。表头没有可信焊缝、标定或同步时仍应报告失效，不降低门槛。Ctrl-C只停止自己启动的9002调试桥；9001、相机和旧安装保留。回退选择install_wireless_rgb或install_wireless再source并重启自己管理的桥接，勿Git reset。当前修复没有新增公网依赖，独立构建可用现有离线归档。
