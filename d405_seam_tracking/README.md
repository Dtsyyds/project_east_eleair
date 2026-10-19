# D405 视觉＋深度融合巡迹实验框架

本目录独立于整车 stable/guided-anchor 路径。CPU 基线、数据抽取、三组回放评测、人工标注导入、SMP 短训练/加载/ONNX 和被动 ROS 适配已有实现；实测结果集中在 [framework_validation.json](reports/framework_validation.json)。不是已训练的焊缝产品，不发布车体轨迹、不控制底盘、不再次持有相机。

本机为 RDK S100P V1P0、aarch64、Ubuntu 22.04.5、ROS Humble、Python 3.10.12。2026-10-04 最新实机 Infra1＋Depth 8 秒处理 235 对，约 29.36 Hz，处理 P95 25.83 ms，latest 覆盖0对；Color＋Depth 8 秒处理179对，约22.34 Hz，P95 40.44 ms，latest 覆盖8对，双方同步过期11/18帧。现场画面均全部 LOST，中央 ROI 深度有效率约1.8%；未证明焊缝精度，也未证明有效几何分支的完整硬件实时能力。此前带上位机overlay的加载场景仅4.56 Hz，25张JPEG实际解码通过，这个低吞吐结果也保留。原速率丢帧计数保留，不使用降速结果替代。

`real_seam_dataset_validated=false`、`trained_seam_model_ready=false`、`algorithm_accuracy_validated=false`、`hardware_realtime_validated=false`、`production_ready=false`。合成训练仅证明管线可执行；不提供焊缝权重。整车历史 SSD 仍不可见，原验收状态不覆盖。

## 架构与配置

`dataset/contracts` → 原图/无损深度/两个源时间/标定 → `geometry` 局部罐壁背景与相对高度 → `pipeline` 证据融合、拟合、源时间状态 → JSONL/ROS结果/可选事件与离线 Rerun。

唯一实验参数源为 [configs/experimental.yaml](configs/experimental.yaml)。中央偏下归一化 ROI `[0.20,0.35,0.80,0.95]` 是实验初值，与工业相机 ROI 无关。平面/二次曲面、深度范围、噪声倍数、覆盖率、冲突门槛和预测寿命集中配置。配置快照是追溯产物，不是第二权威源；不自动缩小 ROI 或放宽阈值。

|输出状态|含义|有效性|
|---|---|---|
|FUSED_CANDIDATE|视觉与相对罐壁结构一致|实验身份确认；现场身份仍未验收|
|VISION_2D|可信视觉，深度无效/无结构|二维可用，metric_valid=false|
|GEOMETRY_CANDIDATE|深度连续结构，视觉不足|几何候选；seam_identity_confirmed=false|
|CONFLICT_2D|两路可信但位置不一致|保留视觉二维并登记冲突，不平均，不发布有效三维|
|PREDICTED|上次接受测量的短期保持|predicted_only=true，不计测量检出，不输出测量三维|
|LOST|没有可接受证据且预测过期|没有有效路径|

预测按图像源时间推进；重复/回退、长间隔、session/标定/分辨率变化清空历史。背景拟合不足/退化明确失败。高度为朝相机正的相对表面距离近似，非绝对余高精度保证。视觉与几何边界分别输出，不强制等宽。整数像素中心、重投影遮挡最近 Z、无效区不补洞，详细契约见 [data_format.md](docs/data_format.md)。

## 离线环境和安装

现有 `.runtime` 为板端 CPU 环境，`.venv` 为隔离的训练/浮点测试环境；均不替换原 core_env。直接运行不需要公网。`offline/` 保存本次需要的 aarch64/CPython3.10 wheels，精确版本与 SHA 位于同一最终报告；已安装 Ubuntu/Humble/RealSense/DNN/OpenCL 库仍是平台依赖，此缓存不是裸机 OS 镜像。训练完整解析版本在 [requirements-training.txt](requirements-training.txt)。隔离环境使用 PyYAML 6.0.3，以满足显式共享库依赖的版本要求；系统及原 core_env 的 PyYAML 不变。不要升级 BSP/SDK。

```bash
cd /home/sunrise/east_eleair
source tank_inspection_system/scripts/env.sh
F="$PWD/d405_seam_tracking"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
"$F/.runtime/bin/d405-seam" preflight
"$F/.runtime/bin/d405-seam" --help
```

重新安装本框架到自己的环境（不会重建旧整车）：

```bash
"$F/.runtime/bin/python" -m pip install --no-index --no-deps --no-build-isolation --force-reinstall "$F"
"$F/.venv/bin/python" -m pip install --no-index --no-deps --no-build-isolation --force-reinstall "$F"
# 必要时恢复隔离训练依赖；本机已装，默认无需重复执行
"$F/.venv/bin/python" -m pip install --no-index --find-links "$F/offline" -r "$F/requirements-training.txt"
```

新建等价环境须使用 `python3 -m venv --system-site-packages 新路径`，先从 offline 安装 setuptools 75.3.0/wheel 0.45.1，再安装本项目及其缓存依赖；训练环境追加 requirements-training.txt。offline 中 tank-vision-inspection wheel 已核对67个源码文件一致，供显式 AsyncLifecycle 依赖使用，不依赖临时 PYTHONPATH。旧 setuptools 59.6 不识别本项目 PEP621，会构造 UNKNOWN；这是失败，不能当安装成功。所有安装应指定隔离环境。

## 无设备的完整闭环

以下输出路径必须尚不存在；合成集不进入源码/wheel，实际使用才创建：

```bash
WORK="$(mktemp -d /tmp/d405-user-XXXXXX)"
"$F/.runtime/bin/d405-seam" fixture --output "$WORK/dataset"
"$F/.runtime/bin/d405-seam" evaluate --index "$WORK/dataset/index.json" \
  --config "$F/configs/experimental.yaml" --mode all --output "$WORK/analysis" --overlays
export HF_HUB_OFFLINE=1 HF_HUB_DISABLE_TELEMETRY=1
"$F/.venv/bin/d405-seam" train --index "$WORK/dataset/index.json" \
  --output "$WORK/train" --epochs 1 --size 64 96 --smoke
"$F/.venv/bin/d405-seam" export --checkpoint "$WORK/train/checkpoint.pt" --output "$WORK/train/seam.onnx"
"$F/.venv/bin/d405-seam" replay --index "$WORK/dataset/index.json" \
  --config "$F/configs/experimental.yaml" --mode vision_only --visual-backend smp \
  --checkpoint "$WORK/train/checkpoint.pt" --output "$WORK/float_replay"
"$F/.runtime/bin/d405-seam" rerun --index "$WORK/dataset/index.json" \
  --results "$WORK/analysis/fused.jsonl" --output "$WORK/seam.rrd"
```

没有 checkpoint 时显式 `visual_backend=smp` 失败并报 model_unavailable；auto 回退会写真实 classical_cpu 与原因。classical 使用局部亮度条带候选，不冒充神经网络。学习模型只用 SMP 0.5.0 的 U-Net＋MobileNetV2，一个 luminance 通道，RGB 显式转灰度，uint8/255、NCHW，encoder_weights=None。训练采用 SMP SoftBCE＋Dice、亮度/对比度/轻模糊，无几何增强。训练/导出/推理预处理一致；输出 logits 只 sigmoid 一次。恢复训练用 `--resume checkpoint.pt`，输入尺寸须一致；不在板端进行长时间实际训练。

## 真实 MCAP 抽取和标注

明确选择 bag，不扫描硬盘。默认实验视觉用 Infra1；Color 必须显式选择 image/info/extrinsics 三组 topic。已验证实际 ROS 类型按 CDR 反序列化；Depth ROS 16UC1 的单位来自既有 RealSense 驱动毫米契约，比例 0.001，与 SDK 原生 depth_units 不混用。

```bash
"$F/.runtime/bin/d405-seam" extract --bag /实际数据/单个D405bag \
  --session 场景会话名 --output /用户指定/新dataset \
  --image-topic /d405/camera/infra1/image_rect_raw \
  --depth-topic /d405/camera/depth/image_rect_raw \
  --image-info-topic /d405/camera/infra1/camera_info \
  --depth-info-topic /d405/camera/depth/camera_info \
  --extrinsics-topic /d405/camera/extrinsics/depth_to_infra1 \
  --depth-scale-m .001 --start-sec 0 --duration-sec 30 --every 3
# 多bag重复 --bag。Color 用 color/image_raw、color/camera_info、depth_to_color。
```

缺 CameraInfo、外参或类型明确失败；不会猜位姿/时间/比例。抽取同步队列有界，expired/unmatched 和抽样分开计数；无法配对的源消息不会伪造配对。正式离线评测对索引中每一帧依次处理，零静默结果丢弃。

标注复用现有人工 GUI，不调用 tracker 生成真值：

```bash
"$F/.runtime/bin/d405-seam" annotate --index /dataset/index.json --sample frame_00000003.npz \
  --prepare --output /用户指定/新label_input
python3 tank_vision_inspection/scripts/annotate_weld_boundaries.py \
  --input /用户指定/新label_input/original.png --output /用户指定/新manual_output
# GUI：1左/2右、鼠标加点、u撤销、s保存；两边各至少6点，原图坐标
"$F/.runtime/bin/d405-seam" annotate --index /dataset/index.json --sample frame_00000003.npz \
  --annotation /用户指定/新manual_output/original_annotation.json --approve --author 操作员姓名 \
  --output /用户指定/新approved_sample
```

导入核验原图 SHA/像素、原图坐标、左右顺序和共同 y 区间；需要人工显式批准。无焊缝用 data_format 中 no_seam 标注。`index.json.samples` 的 session/split 应按采集会话统一登记；同一 session 同时进 train/test 会拒绝。多个 approved_sample 放在同一 dataset 根下，用相对 sample 路径汇入一个索引，保留 SHA 与 annotation；无需重新复制源 MCAP。只有一个 session 时报告独立泛化不足。真实训练不加 --smoke，只处理 approved/human train 样本；验证只用 val，会话分离测试使用 evaluate `--split test`。

## 独立 ROS 使用

```bash
cd /home/sunrise/east_eleair
source tank_inspection_system/scripts/env.sh
F="$PWD/d405_seam_tracking"
export PATH="$F/.runtime/bin:$PATH" OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
python3 -m colcon --log-base "$F/ros2_ws/log" build --base-paths "$F/ros2_ws/src" \
  --build-base "$F/ros2_ws/build" --install-base "$F/ros2_ws/install" --packages-select d405_seam_ros
set +u; source "$F/ros2_ws/install/setup.bash"; set -u
# 当前相机已经运行。此节点只是订阅，不启动驱动。
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ros2 launch d405_seam_ros tracking.launch.py config:="$F/configs/experimental.yaml"
# 需要RGB输入时：追加 stream:=color。不是把RGB resize成深度。
```

有限时间实机分析与显示请求：

```bash
A="/用户可写的分析根/d405_$(date +%Y%m%d_%H%M%S)"
ros2 run d405_seam_ros d405_seam_node --config "$F/configs/experimental.yaml" \
  --stream infra1 --depth-scale-m .001 --duration 60 --output "$A" --overlay --events "$A-events"
# 另一终端 source相同环境；只有此显式请求才复制/写入原图、原始深度和诊断
ros2 service call /vision/d405_seam/save_snapshot std_srvs/srv/Trigger '{}'
ros2 topic echo /vision/d405_seam/diagnostics --once
ros2 topic echo /vision/d405_seam/result --once
```

输入 sensor QoS best_effort/depth5，factory Extrinsics reliable/transient_local/depth1；同步每侧最多6帧，处理持有一对 latest，覆盖计数。vision_only独立处理图像，不要求深度或标定；fused缺标定也保留可信二维但禁三维。结果 `std_msgs/String` 内含完整 JSON 契约，`nav_msgs/Path` 仅有效相机光学坐标米值；无效时发空 Path 清除旧显示，原有效点对应关系在 JSON mask/index 中。`/vision/d405_seam/overlay` 是 BGR8 原图＋状态/概率/原始测量有效区/相对高度小窗；缺项显示unavailable，无效深度保持黑色，拒绝候选红点、预测紫色、冲突红色。仅 `--overlay` 且有订阅者时创建；默认不发完整点云/provenance。事件队列最多3、默认限频1秒/最大30，metadata 最后提交，线程失败/关闭超时不可声称安全关闭。

查看 overlay 可复用现有离线上位机；保留原9001 RGB服务，另一调试桥用9002：

```bash
ros2 run agv_bridge websocket_bridge_node --ros-args \
  --params-file "$PWD/physic_bev/src/system_websocket/agv_bridge/config/tank_inspection.yaml" \
  -p local_server_port:=9002 -p tank_inspection.depth_camera_topic:=/vision/d405_seam/overlay
```

页面连接 `ws://板端IP:9002`，D405视频区域显示实验 overlay；此桥的活性是派生图像活性，不能替代原始设备自检。9001仍显示 RGB。默认 control_enabled=false，不做运动。单机 ROS observer 的计数不等于上位机物理无线验收。

## 加速与性能

CPU 留拟合、时序、ROS；OpenCL 密集核实际执行反投影、mask 和相对表面残差，预分配四个 buffer，上传/下载/同步计入墙钟，非连续输入显式复制。已在 Mali-G78AE 上逐元素核对；没有共享 buffer 零拷贝声明。显式 geometry_backend=opencl 初始化/执行失败直接报错；auto 当前保守选择 CPU，结果写实际执行后端。质量拒绝早退写 cpu_quality_only，不冒称执行 GPU。848×480合成完整几何分支性能与600帧内存观察见最终报告；合成结果不计作金属焊缝实时验收。

显示优化已实际接入：Humble Image.data使用array('B')，消除逐字节Python校验；Brown/pinhole重投影不再分配未使用雅可比；诊断小窗先取相同nearest像素再着色。同一真实帧业务结果完全一致、有效mask完全相同、配准深度最大差1.1e-16m；七种真实/合成画面的overlay逐像素一致。配对处理P95 105.9→73.0ms，小窗绘制P95 15.7→3.1ms。真实Color＋Depth→ROS→9002→本机Firefox：35秒处理420帧、11.98Hz；其中20.2秒解码231帧、11.43FPS，完整回调P95 91.8ms。两秒窗口最低9.77FPS，全部420帧LOST；只证明该负载的机制与平均速度，hardware_realtime_validated仍false。diagnostics/result_manifest新增callback_window_p95_ms、overlay_window_p95_ms、overlay_published，不能把pipeline timing当整条显示链路耗时。preview_rate_fix保留中间7.4FPS失败结果和原先统计，未修改ROI/阈值/拟合/时序配置。当前核心＋MCAP/DDS测试40项通过；与历史测试不累加。

```bash
"$F/.runtime/bin/d405-seam" benchmark --index /dataset/index.json \
  --config "$F/configs/experimental.yaml" --iterations 500 --output /新路径/benchmark.json
cmake -S "$F/native" -B "$F/native/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$F/native/build" -j2
```

HBM adapter 已按本机 DNN/UCP 3.13.6 编译：F32外部输入/输出、按 SDK stride/对齐内存、cache clean/invalidate、task submit/wait/释放，超时保留仍活跃资源并拒绝继续推理。运行需真实焊缝 HBM 与独立模型契约，含 target=nash-m、输入/输出shape、layout、luminance_uint8_div255、HBM SHA、compiler_log/其 SHA、operator_placement、native_library。整数外部 IO 和动态形状明确拒绝，不猜量化。内部量化及 CPU/BPU算子分配必须来自编译日志。当前无焊缝 HBM，状态 implemented_unverified，不是加速验收通过。

ONNX→量化→HBM 转换编译器本机未安装，SMP 模型尚未实际编译，不能承诺可直接部署。PC 使用与本机HBRT匹配的 S 系列工具包，先检查算子和固定 1×1×H×W 输入，再用 train/校准集量化、检查 nash-m 目标和算子分配，输出上面的真实契约，浮点/量化输出及几何差异重新评测。没有工具包就明确阻断，不编造 hb_mapper/YOLO 转换命令或升级板端系统。官方 NV12＋MobileNet 分类 pipeline 不是本框架，目标源码路径 `/app/multimedia_samples/sample_pipeline/single_3dgpu_bpu` 本机不存在；不能直接套 IR/NV12。

## 依赖来源与许可

直接调用 SMP 0.5.0，[commit 420ce84](https://github.com/qubvel-org/segmentation_models.pytorch/tree/420ce84b0c2df0286fa9bb2bd1499eea625c9b33)，轻量模型与损失不重写；不 vendor 整仓、不下载 ImageNet/街景权重。librealsense 已有 2.58.4/commit e15c5d6bb1563e778d116f682aeefffbae2daedc，驱动4.57.7，保留其原许可与唯一 owner。D-Robotics [rdk_model_zoo rdk_s](https://github.com/D-Robotics/rdk_model_zoo/tree/380e1a2bf42041af54be6f34935e50197cfadff9)（Apache-2.0）和本机 `/app/pydev_demo/03_instance_segmentation_sample/01_unetmobilenet` 仅提供API参考，不运行下载型run.sh、不复用街景分类标签；本项目 native/OpenCL 计算为新增代码。

robotic-sealing commit 5879cdd395438c930472ac5fa15663cd1c313696 的 LICENSE 未找到，未复制代码；seam_detection commit 44beaf0bb9339a696c3ed84f3474ed1b3bb4470f GPL-3.0，仅方法参考；DFormer commit e3273009b759b578945483828ff315d560be94c9 保留后续 PC研究，未引入依赖。沟槽密封/平面交线不等于圆柱罐壁凸起焊缝。许可见 [NOTICE](NOTICE)，wheel自带许可保留。复用项目本身能运行不计作本项目效果证据。

## 测试、回退与真实数据最小清单

```bash
source /home/sunrise/east_eleair/tank_inspection_system/scripts/env.sh
F=/home/sunrise/east_eleair/d405_seam_tracking
set +u; source "$F/ros2_ws/install/setup.bash"; set -u
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2 \
  "$F/.runtime/bin/python" -m pytest -q -p no:cacheprovider "$F/tests"
```

独立 ROS package 的 colcon 注册0项测试，功能测试是上面单独执行的 pytest/DDS，不把0项称通过。回退仅 Ctrl-C 停本节点/9002调试桥，新终端只 source 原总控 env；原算法/驱动/生产安装不换，勿用 Git reset。临时测试原图/深度/合成 checkpoint 归档必要摘要后清理，不能作为交付焊缝权重。

后续真实数据最小清单：D405 Infra1（或明确 RGB）和无损 uint16/float Depth；双方 CameraInfo、流间外参、明确 depth_scale；双方原 header和接收/bag时间；分辨率、是否校正/配准、设备序列/工作距离/光照/罐壁与焊缝场景说明；可选人工批准左右边界/mask及无焊缝负样本。Image/CameraInfo/header通常可从现有MCAP读，Extrinsics须实际有录制，缺失时补同设备/配置的真实资料；ROS 16UC1单位可由冻结驱动契约确认。D405→车体外参、独立毫米真值、人工场景说明不能凭图像推断。无需现在立即补采；拿到数据先做 vision_only/depth_only/fused，再决定训练与研究网络。
