# 实际架构与功能状态

`tank_vision_inspection` 是有 HEAD 的 Git 仓库；`physic_bev` 根无 Git，`physic_bev/src` 是无 HEAD/remote 的初始仓库；父目录也无有效 Git。第三方 Livox-SDK2、Sophus、librealsense 是实际独立 Git 仓库。完整初始版本、dirty/untracked 和逐源码 SHA 在 reports/refactor_v1，不伪造 physic 版本。没有发现适用 AGENTS.md。

数据流：`start_vehicle_collection.sh` → `visual_field_validation.launch.py` → 唯一工业相机 driver → image_raw/CameraInfo → hobot_codec → image_h264 → front_h264_recorder → 分段 MKV + frames.jsonl + frame_map。其他分支分别启动 Livox、RealSense 和 Eddy；topic_inventory 实际发现 topic 后建立 core/mid360/d405/eddy 分组 MCAP。monitor、SSD guard 和 PID/starttime 登记贯穿采集。`stop_vehicle_collection.sh` 按登记进程停止 → finalize → 完成/哈希/验收；历史封存 run 不再改写。

回放由多个 BagSource 的 heap 时间轴与 VideoSource 合并，按 bag/原 header 时间发布 `/replay/...`；sensor-only 选择传感器，algorithm-replay 启动既有 stable 节点。RRD 是离线读取，不等于 ROS E2E；role handoff 按单一 profiles 配置生成 manifest、物化、验证。

公共职责仅两处提取：`tank_vision.async_lifecycle` 管理实际重复的线程、队列与 deadline；已安装 `inspection_tools.video_index` 管理解码帧索引，validator/replayer/RRD 统一调用。RRD package.xml 显式依赖 inspection_tools，跨仓库复用不依赖临时 PYTHONPATH。

|功能|当前状态|证据/限制|
|---|---|---|
|工业相机曝光/时间戳/唯一拥有者|已实现，硬件未复测|driver SDK 100000 ns tick，软件锚定 host ROS；采集脚本关闭 tracker 自带相机|
|D405 图像/信息；MID360 点云/IMU|已有接口，硬件未复测|现有 launch + topic_inventory；MCAP 插件阻断完整回放|
|Eddy 原始与均值采集|已实现，硬件未复测|20×8 mV；同步 raw/mean 实际安装版各5条 E2E|
|分组 MCAP、视频分段、SSD/PID|已有编排；视频 fixture 通过|MCAP reader/writer 环境缺失；硬件编码负载需复测|
|手动/自动快照、ROI、provenance/容量|已实现|安装版手动快照 E2E、complete/SHA、ROI/故障/容量单测；自动硬件触发未执行|
|停止、异常关闭、封存、SHA|已修复工程行为|阻塞/异常/重复 close fixtures；封存历史只读|
|sensor-only/algorithm、多 bag、工业视频回放|入口保留，部分验证|统一时间轴代码及视频跨消费者 fixture；真实 MCAP 多 bag 未验证|
|板端 RRD、预览限频、strict/原子提交|fixture 已实现验证|真实 Rerun CLI/CDR fixture；注入 reader 非 MCAP 验收|
|角色 handoff|fixture 已验证|当前 run 动态 RRD 名称、物化/SHA、角色 schema|
|stable/guided-anchor|实验算法保持一致|冻结90个记录结果 SHA 相同，不代表焊缝准确性|
|FAST-LIVO2|已有源码/适配入口，未现场验收|默认 lidar_type=1 订阅 CustomMsg；PointCloud2 另分支，录制格式必须匹配，未改参数|
|FusionIndex|时间关联已实现|原 legacy 保留；native Eddy E2E；坐标有效性 false|
|标定/硬件同步/空间融合|未标定或未证实|CameraInfo K/P 零值保持；粗略 TF 不作有效外参|
|三维激光/模型后端/BEV/测厚/控制|未纳入本任务实现|depth_to_pointcloud 是已有深度投影，不能宣称激光重建或毫米测量|

权威配置路径见 manifests/entrypoints.json；本目录只引用。每次 run 的 frozen_configs 是追溯快照。没有新增空 ROS 包或迁移物理路径。


模型源码状态：inference/onnx_backend.py 和 opencv_backend.py 已有 CPU load/infer 实现，默认 model_path=null、model_calibrated=false，尚无配置模型的推理验收。d_robotics_bpu_backend.py 和 rknn_backend.py 是必须注入 SDK-reviewed adapter 的协议边界，不能将 runtime 模块存在视为加速模型接通。实际依赖发现结果见 model_capabilities.json。以上后端代码和算法配置均未修改。EMAT/RobotState/ProbeState/UltrasoundFrame 的消息或订阅边界不等于真实驱动或运动闭环。
