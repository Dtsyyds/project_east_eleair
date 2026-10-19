# inspection_rerun

`inspection_rerun` 是独立的、低频、允许丢帧的 ROS 2 → Rerun 可视化桥。它只用于
调试和观察，不启动或控制任何相机、雷达、IMU、涡流设备，也不启动 rosbag record。
Rerun 不是数据真源，MCAP 始终是原始数据的唯一真源；关闭 Viewer 或桥接节点不会
影响采集和 MCAP 写盘。

## S100P 安装

环境要求为 Ubuntu 22.04、aarch64、Python 3.10 和 TROS Humble。不要使用 sudo：

```bash
cd ~/east_eleair/tank_vision_inspection
./scripts/setup_rerun_s100p.sh
```

脚本使用 `python3 -m venv --system-site-packages .venv_rerun` 创建独立环境，并固定
安装 `rerun-sdk==0.36.2`。脚本不会修改 `.bashrc`、系统 Python 或 `/opt/tros`。

每个新终端需要：

```bash
source /opt/tros/humble/setup.bash
source ~/east_eleair/tank_vision_inspection/.venv_rerun/bin/activate
source ~/east_eleair/tank_vision_inspection/ros2_ws/install/setup.bash
```

## 构建

```bash
cd ~/east_eleair/tank_vision_inspection/ros2_ws
# 必须由虚拟环境 Python 启动 colcon，使 ROS 节点入口使用 .venv_rerun。
python -m colcon build --symlink-install --packages-select inspection_rerun
source install/setup.bash
```

直接调用系统 `/usr/bin/colcon` 会生成 `/usr/bin/python3` shebang，该解释器看不到仅安装
在 `.venv_rerun` 内的 Rerun SDK，因此本包明确使用 `python -m colcon`。

## 本地 Viewer

只启动桥，不启动传感器和录制：

```bash
ros2 launch inspection_rerun rerun_live.launch.py output_mode:=spawn
```

连接已经运行的 Rerun gRPC 服务或 Viewer：

```bash
ros2 launch inspection_rerun rerun_live.launch.py output_mode:=connect
```

连接地址可在 `config/rerun.yaml` 的 `connect_url` 中修改。

## MCAP 回放

```bash
ros2 launch inspection_rerun rerun_replay.launch.py \
  bag_path:=/absolute/path/to/bag \
  output_mode:=spawn \
  playback_rate:=1.0 \
  loop:=false
```

默认不使用 `--clock`。只有明确需要 ROS 仿真时钟时才设置 `use_clock:=true`。回放
launch 不删除、不修改 MCAP。

## 无 GUI 保存 RRD

自动化和无显示器环境统一使用 save 模式：

```bash
ros2 launch inspection_rerun rerun_live.launch.py \
  output_mode:=save \
  save_path:=$HOME/east_eleair/tank_vision_inspection/output/rerun/live.rrd
```

或者回放短 MCAP：

```bash
ros2 launch inspection_rerun rerun_replay.launch.py \
  bag_path:=/absolute/path/to/small_bag \
  output_mode:=save \
  save_path:=$HOME/east_eleair/tank_vision_inspection/output/rerun/replay.rrd
```

生成后可在有 Viewer 的机器执行 `rerun file.rrd`。

## 订阅与实体

默认配置位于 `config/rerun.yaml`。标准输入包括 Image、CameraInfo、PointCloud2、
Imu、TFMessage、DiagnosticArray、Odometry、Path 和 PoseStamped。若运行环境中可用
`inspection_interfaces`，还会订阅真实定义的 EddyCurrentFrame、UltrasoundFrame、
ProbeState、RobotState 和 FusionIndex；消息包没有 source 时只警告，不影响标准话题。

主要实体路径：

```text
/world/camera/image
/world/lidar
/world/robot
/world/robot/path
/world/probe
/world/bev/image
/world/bev/coverage
/world/camera/detections
/signals/imu/*
/signals/eddy/*
/signals/ultrasound/*
/synchronization/*
/system/diagnostics/*
```

视觉检测和 BEV 仅预留订阅与显示接口，本包不实现物理 BEV 算法。

## 默认性能边界

- 图像最高 2 Hz，在复制 4096×3072 数据之前限频；
- 点云最高 1 Hz；
- 点云最多显示 100000 点，可用 `point_stride` 进一步抽样；
- 涡流/超声显示最高 2 Hz；
- 所有传感器订阅使用小深度队列，慢时允许丢弃旧可视化帧；
- 完整 10 Hz Mono8 图像和原始点云仍由 MCAP 保存。

参数可通过自定义 YAML 修改，也可复制默认 YAML 后启动：

```bash
ros2 launch inspection_rerun rerun_live.launch.py config_file:=/absolute/path/rerun.yaml
```

## 时间线

优先使用 `header.stamp`，以整数纳秒写入 `ros_time`；零时间戳或无 header 时使用
节点接收时间并警告。另有全局递增的 `frame_sequence`。时间回退只产生节流警告，
不会把消息到达顺序冒充为采集时间。

## S100P GUI 降级

S100P 使用 Wayland。若本地 Viewer 因 Wayland、DRM 权限或 GPU 驱动失败：

1. 保留完整终端错误；
2. 确认 `DISPLAY=:0` 和 `WAYLAND_DISPLAY=wayland-0`；
3. 使用 `id -nG` 检查 `video`、`render` 组；
4. 不替换 Mesa/Vulkan/地瓜 GPU 驱动，不污染 `LD_LIBRARY_PATH`；
5. 改用 `output_mode:=save` 生成 `.rrd`，再复制到其他机器查看。

当前用户若不属于 `video`/`render` 组，本包不会自行更改用户组，因为该操作需要
系统权限。

### 本机实测结果（RDK S100P）

`rerun-cli 0.36.2` 的 aarch64 二进制和 Python SDK 均可正常导入，save 模式及真实
MCAP 回放可正常生成 RRD。当前板卡上 Viewer 在 Wayland 和临时 X11
（`env -u WAYLAND_DISPLAY DISPLAY=:0`）两条路径均在 Vulkan `vkCreateInstance`
内部触发 SIGSEGV。用户已经属于 `video`、`render` 组，`card0` 和 `renderD128`
权限正常，因此未修改 Mesa、Vulkan、地瓜 GPU/BPU 驱动或 `LD_LIBRARY_PATH`。
在图形栈兼容问题解决前，请使用 `output_mode:=save`，或使用 `output_mode:=connect`
连接另一台机器上的 Viewer。

## 已知限制

- 现场默认话题为 `/camera/front/image_raw` 与 `/camera/front/camera_info`；历史兼容话题不再默认启用，
  不是深度图；
- CameraInfo 内参全零时只显示二维图像，不创建虚假的 Pinhole，因此无法准确三维投影；
- TF 可以不完整，各个已收到的变换仍会显示，并对非法/空 frame 做防御处理；
- `EddyCurrentFrame` 本身只有 ROS header 时间戳，没有独立的设备原始时间戳、时钟
  漂移或校正偏移字段，本包不会猜测这些值；
- `FusionIndex` 只显示索引、有效状态和时间差，不复制大图、点云或波形；
- BEV coverage/image 暂按 Image 显示，缺陷接口按现有 DefectArray 显示；尚未实现
  物理 BEV 计算；
- Rerun 输出不得作为测量结果、标定结果或质量判定的数据真源。

## 采集完成后的 run 级 RRD

`build_run_rrd` 只用于已经停止、关闭 rosbag 并完成 finalize 的 run。它不创建 ROS
publisher/subscriber，不发布 `/clock`，也不启动 Viewer、传感器或算法。多个 MCAP 由
同一进程中的 `rosbag2_py.SequentialReader` 按记录时间戳堆归并，因此没有 DDS
subscriber drop，也没有多个 bag player 的时钟竞争。

推荐从源码包使用低优先级包装脚本：

```bash
cd ~/east_eleair/tank_vision_inspection/ros2_ws/src/inspection_rerun
scripts/build_run_rrd_nice.sh /media/mcap_data/runs/RUN_ID
```

包装脚本只接受一个参数：已经 finalize 的 run 目录。它自动使用同目录名作为 RUN_ID，
读取包内 `config/rerun.yaml`，并输出到 `RUN_ID/rerun/RUN_ID.rrd`。默认策略为 strict、
ROS_DOMAIN_ID 184、1 Hz JPEG camera preview、7200 秒超时及 nice/ionice 低优先级。
需要自定义参数时使用下面的 Python 高级入口。

若 run 已停止但未通过 complete/finalized 验收，仅为诊断查看可执行：

```bash
scripts/build_run_rrd_incomplete.sh /media/mcap_data/runs/RUN_ID
```

该命令仍执行其余安全检查，输出自动命名为 `RUN_ID_INCOMPLETE.rrd`，并固定标记
`valid_for_formal_validation=false`；它不能把失败 run 变成正式验收产物。若上一次转换
安全中断并留下 partial，再次执行同一命令会先将旧 partial 重命名为带 UTC 时间戳的
`.failed-*` 文件后从头转换，不会把缺失前段数据的 partial 续写或删除。

也可以在正确 source ROS/工作区后直接调用冻结环境中的入口：

```bash
nice -n 10 ionice -c2 -n7 \
  ~/east_eleair/tank_vision_inspection/.venv_rerun/bin/python \
  -m inspection_rerun.run_rrd_builder --help
```

系统 `/usr/bin/python3` 当前没有 `rerun-sdk`，不能替代项目固定的
`.venv_rerun/bin/python`。`--dry-run` 只执行发现、身份、时间范围、SHA、版本、磁盘和
完整性预检。默认拒绝覆盖已有 RRD；只有显式 `--force` 才允许在最终原子提交时替换
同名派生产物。

strict 模式要求 `complete=true`、`finalized=true`、manifest run ID 一致、MCAP validation
通过、登记的 writer/bag 存在、必要消息类型可加载、各 bag 时间范围相容、最多一个
记录的 `/clock`、源 SHA 可解析、Rerun SDK/CLI 精确为 0.36.2，并且输出磁盘有余量。
`allow-incomplete` 会把输出自动改成 `*_INCOMPLETE.rrd`，根实体写入警告，报告固定
`valid_for_formal_validation=false` 并列出所有绕过门槛。

preview 相机策略优先选择 MCAP 中的 front Image；若不存在，则读取
`video/front/*.mkv` 与同名 `*.frames.jsonl`，按 ROS header 时间最高 1 Hz 抽帧，等比
缩放到最大宽 1280，并以 JPEG quality 85 写入 `EncodedImage`。两种来源同时存在时只
选 MCAP，避免重复画面。`camera-mode=none` 完全不打开视频。

处理时只创建 `RUN_ID.rrd.partial`。writer 正常关闭、0.36.2 CLI footer 校验、实体和
topic 计数检查、失败率检查及 SHA256 成功后，才通过 `os.replace` 原子提交 `.rrd`。
输出目录同时包含：

```text
RUN_ID.rrd
RUN_ID.rrd.sha256
rrd_manifest.json
rrd_validation.json
conversion.log
```

PC 必须安装完全相同的 Viewer 版本：

```bash
cd /data/rerun/RUN_ID
sha256sum -c RUN_ID.rrd.sha256
rerun --version                 # 必须为 rerun-cli 0.36.2
rerun RUN_ID.rrd
```

只传派生产物和报告的示例（本工具不会自动连接 PC）：

```bash
rsync -ah --partial --append-verify --info=progress2 \
  RUN_ID.rrd RUN_ID.rrd.sha256 rrd_manifest.json rrd_validation.json conversion.log \
  user@pc:/data/rerun/RUN_ID/
```

RRD 成功绝不表示源 MCAP、MKV、sidecar、manifest 或校验报告可以删除。
