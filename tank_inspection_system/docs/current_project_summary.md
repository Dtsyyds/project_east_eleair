# 当前罐体巡检工程功能、框架与进度总结

整理日期：2026-10-04。依据源码主调用链、配置、两份最终报告、安装文件SHA及当次ROS运行状态进行只读核查。运行状态为核查时快照，后续可能变化。

目前软件采集与离线处理链路已跑通，无线RGB显示已接通，D405融合实验框架可运行；真实焊缝效果和整车现场验收尚未完成。

## 一、工程分工

| 工程 | 主要职责 | 当前进度 |
|---|---|---|
| tank_inspection_system | 统一环境、独立构建、采集/停止/校验/回放/交付入口、操作文档、上位机页面 | 已实现，保留原入口及回退安装 |
| tank_vision_inspection | 工业相机视觉、stable/guided-anchor、实时巡迹、精细检查、快照、ROI记录、Rerun | 工程兼容性已验证，现场算法精度未验收 |
| physic_bev | D405/MID360/涡流等驱动、采集编排、FAST-LIVO2适配、时间同步、WebSocket桥 | 软件接口与fixture通过；本阶段实际接通验证的是D405 |
| d405_seam_tracking | D405图像与深度配准、罐壁几何、证据融合、时序、训练、评测、独立ROS输出 | CPU实验闭环已实现，真实数据与有效焊缝模型待补齐 |

实际平台为RDK S100P、aarch64、Ubuntu 22.04.5、ROS 2 Humble。tank_vision_inspection有Git HEAD及既有修改；physic_bev/src为尚无HEAD的仓库；不能将父目录或各工作空间假定为同一个Git仓库。

## 二、主要数据流

```text
传感器唯一驱动 → ROS topic
                  ├─ 分组MCAP / 工业视频 / 快照
                  │    → 停止封存 → SHA校验 → 回放 / RRD / 角色交付
                  ├─ WebSocket桥 → 离线上位机RGB、设备状态、雷达位姿
                  └─ 工业视觉或D405实验 → 结果、轨迹、诊断、事件证据
```

总控包装脚本引用原工程入口，透传参数、退出码和信号。权威配置保留在所属工程；run中的冻结配置用于追溯。新安装与原安装分开，回退依靠选择安装空间和停止独立实验节点。

## 三、已实现的工程功能

- 采集与证据：分组MCAP、分段MKV、帧时间索引、SSD保护、进程登记、手动/自动快照、ROI记录、容量限制。
- 回放与交付：sensor-only/algorithm模式、多bag时间轴、工业视频回放、严格源文件SHA、RRD原子提交、按角色物化与校验。
- 工程修复：异步异常传播、关闭deadline、有界队列和统计、严格视频帧映射、时间戳有效性、涡流原始/均值适配、动态handoff命名。
- 无线功能：设备消息活性自检、两路JPEG接口、Odometry位姿、断线重连、可选控制转发与watchdog。当前底盘控制关闭；鱼眼、EMAT、底盘topic尚未配置完整。

MCAP插件早期失败已解决，实际写入、关闭、读取、反序列化及相关fixture已通过。历史SSD仍不可见，历史run实际SHA和完整源数据审计尚未完成。

WebSocket自检证明近期有效ROS消息存在，不代表设备全部故障诊断或物理执行确认。FusionIndex完成时间关联，不等于空间融合已标定。FAST-LIVO2有源码与适配入口，尚未完成本阶段现场验收。

## 四、D405融合实验框架

当前采用证据级融合：视觉提出候选，深度估计局部平面或二次罐壁，并计算相对高度。两路一致时接受，冲突时降级，深度失效时保留可靠二维结果，短期预测单独标记。

支持vision_only、depth_only、fused三组评价。三维结果使用相机光学坐标和米单位，尚无有效D405到车体外参。图像/深度源时间戳分别保留，配准使用真实标定、畸变和外参，遮挡选择最近深度，不以resize代替配准。

| 状态 | 含义 |
|---|---|
| FUSED_CANDIDATE | 视觉与几何证据一致，仍属于实验候选 |
| VISION_2D | 视觉二维有效，测量型三维无效 |
| GEOMETRY_CANDIDATE | 几何结构存在，焊缝身份尚未确认 |
| CONFLICT_2D | 两路可信但冲突，保留二维并登记原因 |
| PREDICTED | 源时间限定的历史预测，不计为测量检出 |
| LOST | 当前没有可接受证据或预测已过期 |

训练管线使用SMP 0.5.0的U-Net加MobileNetV2、单通道输入。短训练、checkpoint恢复、推理和ONNX导出已通过合成测试。当前实际视觉后端仍是classical_cpu，没有可交付的真实焊缝权重。

OpenCL密集运算已在Mali-G78AE验证；当前实机融合测试使用CPU。BPU适配已编译，但缺匹配编译工具链和焊缝HBM，尚未完成模型部署验证。没有全链路零拷贝或全BPU运行的验收结论。

框架提供preflight、fixture、extract、annotate、replay、evaluate、train、export、benchmark、rerun等CLI入口，以及独立ROS适配。ROS订阅既有D405驱动，输出结果、相机坐标三维轨迹、diagnostics及可选overlay/事件快照。

## 五、验证进度

本次只读核查确认：

- 97个原工程源码/安装文件SHA仍一致。
- D405两个隔离环境共24个源码/安装文件SHA一致。
- 最新修改文件SHA与交付记录一致，两份报告之间的SHA引用一致。
- 最近测试记录为40项框架测试、10项无线/浏览器测试通过；重叠组不累加。
- 原13个入口通过记录仍保留，本次未重新执行全量测试。

最近一次实测性能：

| 项目 | 结果 |
|---|---:|
| RGB预览 | 14.94 FPS |
| RGB加深度融合处理 | 11.98 Hz |
| 浏览器融合画面显示 | 平均11.43 FPS |
| 完整回调P95 | 91.8 ms |
| 最低两秒显示窗口 | 9.77 FPS |

平均速度已超过10Hz，但严格持续10Hz尚未保证。该轮420帧全部为LOST，不能作为焊缝检测有效性的证明。最近场景的ROI深度有效率中位数约20.3%；早期报告中约1.8%的数值属于另一轮场景，不应混用。

## 六、核查时实际运行状态

当前ROS域0中看到/d405/camera和/websocket_bridge_node。Color与depth_module.global_time_enabled均为true。

- 9001：WebSocket原始RGB预览，使用install_wireless_preview。
- 8080：离线上位机页面服务。
- D405融合节点与9002调试桥没有常驻运行，需要使用独立入口启动。
- 已有D405驱动为唯一相机owner。

页面地址为http://192.168.1.6:8080/，另一网口为http://192.168.127.10:8080/；实际访问以局域网可达地址为准。页面分别统计收到FPS与成功解码显示FPS。

## 七、尚未完成及后续顺序

1. 真实焊缝及无焊缝数据、人工标注和三组消融评价：明确当前LOST的场景原因，验证准确性与泛化能力。
2. SSD与实际源数据：恢复介质可见性，只读审计历史封存run的库存及实际SHA，再开展新run采集复测。
3. 其他硬件接入：补齐鱼眼、EMAT、底盘等实际topic、消息、单位和运行契约。EMAT已有独立串口启动/清零/单次采集代码及测厚研究脚本，但尚未接入当前ROS/WebSocket链路，测厚结果未完成现场验收。
4. 标定与稳定性：完成D405到车体外参、有效焊缝负载下的持续性能、物理无线和整车稳定性验证。
5. 板端模型部署：真实训练与评价通过后，使用匹配S100P的工具链验证SMP网络编译、量化误差和BPU算子分配。

工业相机有效内参、跨传感器rig标定、共同硬件同步尚未验收。工业相机跨传感器融合、MID360空间融合、整罐BEV、机器人运动闭环不属于已完成能力。

```json
{
  "framework_ready": true,
  "training_pipeline_verified": true,
  "ros_offline_e2e_verified": true,
  "real_seam_dataset_validated": false,
  "trained_seam_model_ready": false,
  "algorithm_accuracy_validated": false,
  "hardware_realtime_validated": false,
  "hardware_revalidated": false,
  "algorithm_field_validation_pending": true,
  "production_ready": false
}
```

## 八、权威配置、文档与证据入口

- 无线配置：../../physic_bev/src/system_websocket/agv_bridge/config/tank_inspection.yaml
- D405实验配置：../../d405_seam_tracking/configs/experimental.yaml
- 模块接入与调试手册：[module_development_and_debugging.md](module_development_and_debugging.md)
- 采集、停止、回放、交付及60秒复测：[collection_and_replay.md](collection_and_replay.md)
- D405完整数据、标注、训练、评价、ROS命令：[D405 README](../../d405_seam_tracking/README.md)
- 总控报告：[final_report.json](../reports/refactor_v1/final_report.json)，最新阶段看closeout。
- D405报告：[framework_validation.json](../../d405_seam_tracking/reports/framework_validation.json)，显示修复阶段看preview_rate_fix。

旧README和历史报告保留了早期平台、失败与测试数字。应按报告阶段、源码和实际运行环境判断，不能把旧状态直接当作当前状态，也不能累加重叠测试组。
