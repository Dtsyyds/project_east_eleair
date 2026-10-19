# S0-B S100P 无硬件软件基线

**验证日期：** 2026-09-03
**工程目录：** `/home/sunrise/east_eleair/tank_vision_inspection`
**关联 HEAD：** `33e59417a17e764a127c7d91be1b5098f9a348ba`
**验证性质：** 无硬件软件验证；不包含相机、雷达、BPU、机器人或整套系统运行。

## 1. 结论摘要

本次 S0-B 验证确认了板端软件环境和一组离线合成样本的可运行边界：

- 板端为 S100P，Ubuntu 22.04，系统 Python 3.10.12。
- ROS 2 Humble 和 TROS Humble 可用。
- OpenCV 4.11.0、NumPy 1.26.4、PyYAML 5.4.1 已被验证环境使用。
- `tank_vision_core`、`tank_vision_interfaces`、`tank_vision_ros` 使用独立的 `build_s100p_humble`、`install_s100p_humble`、`log_s100p_humble` 目录构建；ROS 包前缀指向当前板端新目录。
- 13 个测试文件的 JUnit 结果为 44 passed，耗时约 3.827 秒；这是选定的无硬件测试集合，不是全工程回归。
- 8 张合成样本均离线处理成功，生成 8 个 JSON、8 张 mask、8 张 overlay、`summary.csv` 和 `performance_summary.json`。
- CSV 实际记录 6 张 `seam_detected=True`、2 张 `seam_detected=False`，异常候选总数为 2；性能 JSON 实际记录平均算法耗时 114.586986 ms，最大算法耗时 251.461408 ms。
- 本次验证没有证明真实相机帧率、传感器同步、标定、BPU 推理、真实 BEV、Rerun 运行或机器人运动能力。

本文件只记录 S0-B 软件基线，不定义或启动下一阶段任务。

## 2. 证据范围与来源

实验目录：

```text
/home/sunrise/east_eleair/tank_vision_inspection/output/s100p_offline_demo_N1DdzU
```

本次实际读取的证据：

| 证据 | 实际读取结果 |
|---|---|
| `software_baseline.junit.xml` | `tests=44`、`errors=0`、`failures=0`、`skipped=0`、`time=3.827`；时间戳为 `2026-09-03T15:37:54.892808` |
| `results/summary.csv` | 8 行样本结果；逐行核对 seam、异常候选、valid 标志和耗时字段 |
| `results/performance_summary.json` | `processed_frames=8`、`skipped_frames=0`、`dropped_frames=0`、`failed_frames=0`；核对平均值、分位数和最大值 |
| `results/json/` | 8 个 JSON 文件 |
| `results/masks/` | 8 个 mask 文件 |
| `results/overlays/` | 8 个 overlay 文件 |
| `ros2_ws/log_s100p_humble/` | 读取构建日志，确认使用的独立构建、安装和日志目录及包选择 |
| `ros2_ws/install_s100p_humble/` | 在干净 Humble/TROS 子 shell 中检查包前缀和 Python 导入来源 |

以下内容来自用户提供的验证终端记录，本轮未重新执行对应测试：

- S100P、Ubuntu 22.04、系统 Python 3.10.12；
- ROS 2 Humble、TROS Humble；
- OpenCV 4.11.0、NumPy 1.26.4、PyYAML 5.4.1；
- 三个 ROS 包构建成功；
- 四类消息 `Defect`、`DefectArray`、`SeamState`、`VisionQuality` 序列化往返通过；
- 8 张合成样本处理退出码为 0；
- “检出焊缝 6 张、异常候选 2 个”的摘要。

其中，8 帧结果、44 项 JUnit 数量、3.827 秒、性能均值和输出文件数量已经通过本轮实际读取再次核对。构建过程本身没有在本轮重新运行。

## 3. 离线结果核对

### 3.1 JUnit

`software_baseline.junit.xml` 实际内容：

```text
tests=44
errors=0
failures=0
skipped=0
time=3.827
```

XML 中列出了 13 个测试文件对应的测试用例，覆盖配置、中心线、图像质量、几何、序列化、时间戳、指标、Pipeline、分块推理、激光、异步写盘、数据集工具和三通路队列策略。

这 44 项是选定的无硬件测试，不是全工程、真实设备或生产验收。

### 3.2 样本结果

`results/summary.csv` 实际为 8 行数据：

| 项目 | 实际读取结果 |
|---|---:|
| 输入样本 | 8 |
| `seam_detected=True` | 6 |
| `seam_detected=False` | 2 |
| `defect_count` 总和 | 2 |
| `valid_for_measurement=True` | 7 |
| `valid_for_measurement=False` | 1 |
| 结果 JSON | 8 |
| mask | 8 |
| overlay | 8 |
| 丢帧/跳过/失败 | 0 / 0 / 0 |

6 张检测到焊缝的样本包括直线、弯曲、噪声、低对比度、局部遮挡和表面异常样本。两个异常候选分别出现在 `07_surface_anomaly.png` 和 `08_blue_laser.png` 对应结果中，不能算作两个正确缺陷。

`06_no_seam.png` 的结果为 `seam_detected=False`，但 CSV 中仍存在偏差数值。`seam_detected=False` 时，即使存在偏差数值，也不能用于循迹控制。

### 3.3 性能统计

`results/performance_summary.json` 实际读取结果：

```text
processed_frames:       8
skipped_frames:         0
dropped_frames:         0
failed_frames:          0
average_algorithm_fps:  8.726994501167672
average_end_to_end_fps: 7.75397834020943
```

`total_algorithm_ms`：

```text
mean: 114.586986375 ms
p50:  103.2547495 ms
p90:  152.4510826 ms
p95:  201.9562453 ms
max:  251.461408 ms
```

首帧 251.461408 ms 的数值来自 `summary.csv` 第一行；它与性能 JSON 的最大值一致。平均算法耗时 114.59 ms 和首帧 251.46 ms 仅是这 8 张样本的统计，不代表三通路性能、相机帧率或正式 P95 验收。

文件夹图像时间戳来自 `st_mtime_ns`。同一时间戳不代表相机重复帧，不能仅靠时间戳标识唯一帧；本次 `07_surface_anomaly.png` 和 `08_blue_laser.png` 的 CSV 时间戳相同就是一个具体例子。

## 4. 软件构建和导入基线

### 4.1 独立构建目录

验证关联的目录为：

```text
ros2_ws/build_s100p_humble
ros2_ws/install_s100p_humble
ros2_ws/log_s100p_humble
```

构建日志实际记录的核心命令参数为：

```text
colcon --log-base log_s100p_humble build
  --base-paths src
  --build-base build_s100p_humble
  --install-base install_s100p_humble
  --symlink-install
  --executor sequential
  --packages-select tank_vision_core tank_vision_interfaces tank_vision_ros
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

三个 ROS 包的安装前缀在独立子 shell 中实际查询为：

```text
/home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/tank_vision_core
/home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/tank_vision_interfaces
/home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/tank_vision_ros
```

### 4.2 Python 导入来源差异

在如下独立环境中检查：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
```

本轮实际检查结果：

| 模块 | 实际来源 |
|---|---|
| `tank_vision_interfaces` | `/home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/tank_vision_interfaces/local/lib/python3.10/dist-packages/tank_vision_interfaces/__init__.py` |
| `tank_vision_ros` | `/home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/build_s100p_humble/tank_vision_ros/tank_vision_ros/__init__.py`，由新安装目录 egg-link 指向 |
| `tank_vision_core` | 未作为 Python 模块找到 |

因此，“三个包安装前缀均指向板端新目录”已由本轮核对；但“三个包均可 Python 导入”这一概括与本轮实际 Python 检查不完全一致。`tank_vision_core` 是 ament CMake 包，未作为 Python 模块导入不单独等于 ROS 包构建失败；文档保留该差异，不修改安装产物或原始记录。

## 5. 运行环境说明

### 5.1 适用于新终端的干净 Humble/TROS 环境

以下命令用于后续复核，不代表本轮重新运行了整套测试：

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection

env -i HOME=/home/sunrise USER="$USER" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    source /opt/ros/humble/setup.bash
    source /opt/tros/humble/setup.bash
    source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
    ros2 pkg prefix tank_vision_core
    ros2 pkg prefix tank_vision_interfaces
    ros2 pkg prefix tank_vision_ros
  '
```

注意：

- 不要加载旧的 `ros2_ws/install/setup.bash`。
- 不要使用历史机器路径 `/home/dts/visoin_track/tank_vision_inspection`。
- 不要加载 Jazzy；本基线使用 `/opt/ros/humble` 和 `/opt/tros/humble`。
- 若需要使用 Python 包，应在同一个子 shell 内完成环境加载后再执行。

### 5.2 独立目录构建命令

若未来需要重建，应在确认工作树快照和依赖版本后使用独立目录：

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws

source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash

colcon --log-base log_s100p_humble build \
  --base-paths src \
  --build-base build_s100p_humble \
  --install-base install_s100p_humble \
  --symlink-install \
  --executor sequential \
  --packages-select tank_vision_core tank_vision_interfaces tank_vision_ros \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
```

该命令会写入构建、安装和日志目录，本轮未执行。它也不能保证完全可复现，因为源码工作树尚未完整封存，依赖和系统环境仍可能变化。

### 5.3 选定无硬件测试命令

后续若要复核本次选定测试集合，可使用明确的输出路径；以下为候选复核命令，不是本轮执行记录：

```bash
cd /home/sunrise/east_eleair/tank_vision_inspection

source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash

/usr/bin/python3 -m pytest \
  tests/test_config.py \
  tests/test_centerline.py \
  tests/test_image_quality.py \
  tests/test_seam_geometry.py \
  tests/test_serialization.py \
  tests/test_timestamp.py \
  tests/test_metrics.py \
  tests/test_pipeline.py \
  tests/test_tiled_inference.py \
  tests/test_laser.py \
  tests/test_async_writer.py \
  tests/test_dataset_tools.py \
  tests/test_three_paths.py \
  --junitxml=/home/sunrise/east_eleair/tank_vision_inspection/output/s100p_offline_demo_N1DdzU/software_baseline.junit.xml
```

该命令会覆盖指定 JUnit 输出文件，故本轮没有执行。若要保留原始证据，应先使用新的明确输出路径；不能将本命令表述为已保证完全复现原始运行。

## 6. 明确限制

1. 44 项是选定的无硬件测试，不是全工程、真实设备或生产验收。
2. 文件夹图像时间戳来自 `st_mtime_ns`，同时间戳不代表相机重复帧，不能仅靠时间戳标识唯一帧。
3. `valid_for_measurement` 当前只排除严重欠曝、过曝和模糊，不包含标定有效性或毫米测量保证。
4. `seam_detected=False` 时，即使存在偏差数值，也不能用于循迹控制。
5. 两个异常候选分别出现在表面异常图和蓝色激光图，不能算作两个正确缺陷。
6. 实时队列 `drop_oldest` 是主动淘汰过时帧，不等于传感器或 MCAP 录制丢帧。
7. 8 张图的算法耗时不代表三通路性能、相机帧率或正式 P95 验收。
8. 用户已确认相机及 USB3.0 连接；设备目前不在场，本轮未进行采集复测。
9. Rerun 包本轮未重建；BPU、真实同步、标定、真实 BEV 和机器人运动均未验证。
10. 无硬件离线样本不能证明 MindVision SDK 当前连接状态、真实设备时间戳、USB 协商速度或相机帧率。
11. 输出结果依赖未提交源码、当前配置、系统 Python、ROS/TROS 环境和当前安装目录；不能把 HEAD 与三项哈希单独解释为完整软件封存。

## 7. Git 状态与封存边界

本轮检查到的仓库状态：

```text
仓库：/home/sunrise/east_eleair/tank_vision_inspection
分支：main
HEAD：33e59417a17e764a127c7d91be1b5098f9a348ba
状态：工作树未提交
```

当前修改和未跟踪内容按性质分组如下：

### 源码和测试修改

```text
.gitignore
README.md
scripts/run_three_path_demo.py
scripts/test_mindvision_camera.py
tank_vision/cli.py
tank_vision/config.py
tank_vision/sources/vendor_camera_source.py
tests/test_sources.py
```

### 相机配置和采集证据

```text
configs/camera_mv_suf1205gm.yaml
configs/camera_mv_suf1205gm_usb2.yaml
camera_test/
```

### Rerun 源码和脚本

```text
ros2_ws/src/inspection_rerun/
scripts/setup_rerun_s100p.sh
```

### 本次独立构建产物

```text
ros2_ws/build_s100p_humble/
ros2_ws/install_s100p_humble/
ros2_ws/log_s100p_humble/
```

工作树未提交，源码快照未完整封存。HEAD 和以下三个哈希不足以证明全部源码已冻结：

- `configs/default.yaml`
- `output/s100p_offline_demo_N1DdzU/software_baseline.junit.xml`
- `output/s100p_offline_demo_N1DdzU/results/summary.csv`

本轮没有执行任何 Git 写操作，也没有修改 `.gitignore`。

## 8. SHA256 核对

本轮重新读取并计算了三项 SHA256，结果全部与预期一致：

| 文件 | 预期 SHA256 | 本轮计算结果 | 结果 |
|---|---|---|---|
| `configs/default.yaml` | `c5dae7f2b76094715764b89f688fc7c21fa1ba065ef8e30ddab1e17fa14b2b27` | `c5dae7f2b76094715764b89f688fc7c21fa1ba065ef8e30ddab1e17fa14b2b27` | 一致 |
| `output/s100p_offline_demo_N1DdzU/software_baseline.junit.xml` | `843eadd57ae1fc9c518924e19a420afa9722b55db5cd74f5f8f53029bd9ca46e` | `843eadd57ae1fc9c518924e19a420afa9722b55db5cd74f5f8f53029bd9ca46e` | 一致 |
| `output/s100p_offline_demo_N1DdzU/results/summary.csv` | `a80e9befc8ebcc3bef8535b15ab7dd163cd73f5d79d799df301257c64f57c7df` | `a80e9befc8ebcc3bef8535b15ab7dd163cd73f5d79d799df301257c64f57c7df` | 一致 |

没有修改任何文件来匹配哈希，也没有因为哈希差异暂停定稿。

## 9. 本轮执行确认

本轮执行了：

- 适用目录内 `AGENTS.md` 查找；未发现适用文件；
- Git 根目录、分支、HEAD、工作树状态检查；
- 指定实验目录的文件清单和证据文件读取；
- JUnit XML、CSV、性能 JSON 的实际读取和统计核对；
- 三项 SHA256 重新计算；
- 独立构建日志、安装目录、包前缀和 Python 导入来源的只读检查。

本轮没有执行：

- 相机、雷达、BPU、Rerun、机器人或其他硬件访问；
- 硬件采集、ROS 节点运行、整套测试或重新生成实验输出；
- 依赖安装或升级；
- 源码、配置、`.gitignore`、原始证据或构建产物修改；
- Git 暂存、提交、分支切换、stash、reset、clean 或 pull。

本次仅新增本文件；源码、原始实验结果和 Git 暂存区未修改。
