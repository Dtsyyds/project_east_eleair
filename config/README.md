# 配置入口和覆盖规则

1. `deploy/env.sh [s100p|x86|rk3588]` 选择构建/运行平台；包选择来自 `platforms/<platform>.json`，视觉算法平台参数另在 `vision/platforms/`，职责不同。
2. 默认系统运行配置 `communication/system.yaml`。`INSPECTION_CONFIG_ROOT` 可指向另一套完整配置树；`INSPECTION_SYSTEM_CONFIG` 或 `deploy/run.sh --config FILE` 明确替换系统 YAML；后者优先。没有隐含深合并。
3. `FIELD_DATA_ROOT` 显式覆盖 system.yaml 的 data_root；其默认 `/media/mcap_data` 是外部数据盘，不是旧代码路径。ROS launch 参数/节点参数的显式值覆盖其包内默认配置，保留 ROS 原行为。
4. `collection/data_handoff_profiles.yaml` 为交付入口；`vision/`、`d405/` 分别保留现有算法默认值；视觉配置中历史 `configs/foo.json` 按当前配置文件所在目录解析 foo.json，配置字节和封存哈希保持不变。
5. 各 ROS 包的原生配置仍在 `ros2_ws/src/**/<package>/config`，随该包安装到 share；完整目录索引见 `ros_config_catalog.json`。这些目录是对应配置的唯一源码，不在本目录复制第二套。视觉 ROS 专用 defaults 在 `vision/ros`，其语义不同于算法 defaults。

只读资源与生成物：中央视觉资源通过 tank_vision_ros/setup.py 安装到 share/tank_vision_ros/config；修改权威源后重建。不要人工改 install 下资源。
新增运行产物在 `.runtime/<platform>`。采集数据和封存 run 的 frozen_configs 留在外部 data_root；不改写历史 run。
`FIELD_VISION_EVIDENCE_ROOT` 可指向原封存视觉证据的只读数据根（默认新 runtime/analysis）；不会复制大型原 output。
历史复现脚本 `tests/system/*reproduction.py`、frozen_comparison.py 需要显式 `INSPECTION_BASELINE_FIXTURES`，按原相对路径放置历史源码快照；输出写新 runtime/historical_reproduction，必要的旧 final_report.json 也应作为明确的复现输入准备。

`camera/root` 与 `camera/physic` 保留 SDK mvdat，不等同于有效相机内参。rough 外参仍在 inspection_bringup/config/rough_extrinsics.yaml，production_valid=false；其三张证据图片缺失。默认模型路径 null，不代表模型后端可用。

正式 MindVision 适配在 CameraInit 前通过厂商头文件声明的 CameraSetDataDirectory 将 SDK 生成物放入 runtime/mindvision；初始化后按真实序列号从中央 camera/{root,physic}/Data 读取原 mvdat，再应用已有显式节点参数。相同序列号有不同内容时拒绝歧义，使用 MINDVISION_PARAMETER_FILE 明确选择。未找到该序列号文件时保留 SDK 初始参数，不伪造标定。该路径修复的设备实际读取/写入行为仍需下一轮验证；厂商演示程序保留原文与 CWD 行为。
