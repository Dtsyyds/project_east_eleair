# 标定与算法能力状态

工业相机 node.py 的 CameraInfo K/P 继续为零：未标定。未提供经设备序列号、分辨率和内容核验的真实标定文件，因此本任务没有填充有效内参。D405 发布设备 CameraInfo 只能说明设备自身参数来源，不能证明跨传感器 rig 标定。

工业 SDK tick 软件锚定 host ROS，Eddy 为 host_receive，MID360 时间来自 Livox 驱动，D405 受设备/global_time 参数影响。不存在已经验证的共同硬件触发/PTP结论。冻结参数和源码可审计；硬件时间偏差尚待复测。

粗略外参/TF、FusionIndex 的时间关联和 pose interpolation 不能作为空间精度证明。native Eddy mean 适配只有时间索引，没有 I/Q、gain、liftoff 或 sampling rate 的虚构赋值。asset_coordinate_valid=false fixture 已验证。

stable/guided-anchor 是现有实验算法，本次不修改 ROI/reference/unary/DP/IRLS/temporal/模型。ONNX/BPU/RKNN、BEV、激光三维、测厚、毫米测量和机器人运动闭环未增加实现。


模型源码状态：inference/onnx_backend.py 和 opencv_backend.py 已有 CPU load/infer 实现，默认 model_path=null、model_calibrated=false，尚无配置模型的推理验收。d_robotics_bpu_backend.py 和 rknn_backend.py 是必须注入 SDK-reviewed adapter 的协议边界，不能将 runtime 模块存在视为加速模型接通。实际依赖发现结果见 model_capabilities.json。以上后端代码和算法配置均未修改。EMAT/RobotState/ProbeState/UltrasoundFrame 的消息或订阅边界不等于真实驱动或运动闭环。

D405 2026-10-04独立实机读到了设备factory CameraInfo及Depth→Infra1/Color外参；不代表D405→车体/工业/Livox现场标定。ROS16UC1毫米单位已从当前驱动源码核对。新D405实验只发布相机optical frame的米制有效点；缺factory内参/外参降二维，body_coordinate_valid=false。现有工业K/P零值与粗略TF仍保持原状态。
