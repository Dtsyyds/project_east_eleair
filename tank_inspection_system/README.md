# 罐体巡检兼容总控 V1

本目录引用现有 `tank_vision_inspection` 和 `physic_bev`，不复制 ROS 包、算法或权威配置。真实平台为 RDK S100P V1P0 / aarch64 / Ubuntu 22.04.5 / ROS 2 Humble。原安装空间保持不变，新安装空间全部在本目录。

```bash
cd /home/sunrise/east_eleair
bash tank_inspection_system/scripts/build.sh
source tank_inspection_system/scripts/env.sh
bash tank_inspection_system/scripts/preflight.sh
bash tank_inspection_system/scripts/verify_offline.sh
```

本次自检已恢复 MCAP：依赖在本地 offline 目录，实际写入/关闭/读取/CDR、原失败 RRD 测试及原速率 ROS 多 bag fixture 已通过。preflight 仍返回 2，因为 `/media/mcap_data` 不可见；不能据此启动正式采集。历史报告保留，最新结果集中在最终报告的 `closeout` 字段，分组测试不累加。独立通过的测试、安装版 ROS fixture、算法一致性和性能证据见 [最终报告](reports/refactor_v1/final_report.json) 与 [问题证据](reports/refactor_v1/issue_matrix.json)。

硬件短测命令见 [collection_and_replay](docs/collection_and_replay.md)。重构自检阶段未启动真实设备；2026-10-04 用户接入 D405 后的单设备实测另记在 `closeout.d405_hardware_validation`，不覆盖原阶段结论。常用包装入口保留原参数、退出码和 exec 信号语义：`collect.sh`、`stop.sh`、`replay.sh`、`validate.sh`、`handoff.sh`。唯一额外配置为环境变量中的目录引用。

- [模块开发、验证与调试完整手册](docs/module_development_and_debugging.md) / [D405融合实验框架](../d405_seam_tracking/README.md)
- [架构与状态](docs/architecture.md) / [入口](docs/entrypoints.md) / [topic 契约](docs/topic_contract.md)
- [数据契约](docs/data_contract.md) / [标定状态](docs/calibration_status.md)
- [部署与回退](docs/deployment.md) / [交付](docs/handoff.md) / [排障](docs/troubleshooting.md)
- [迁移清单](reports/refactor_v1/migration_map.csv) / [初始 Git 状态](reports/refactor_v1/repositories_before.json)

兼容重构V1没有新增焊缝精度、BEV、测厚或运动闭环实现；随后新增的D405局部几何融合仅是独立实验框架，现场准确性未验收。实验算法的离线一致性不能替代现场准确性验收。

无线入口复用实际存在的 `physic_bev/src/system_websocket/agv_bridge` 与 `agv_protocol`。当前源码和安装均未发现 `agv_bringup`；旧机器人引用的名称不等于该包已交付。新增 `tank_inspection_robot` 被动订阅已有设备，启动不会创建相机 owner、mapping 或正式采集。支持六设备 ROS 数据自检、鱼眼/深度双路限频 JPEG、雷达 Odometry 位姿及可选底盘命令转发。自检是消息活性证据，未知驱动、未配置或无有效消息明确失败/未验证。鱼眼、EMAT、底盘实际接口仍需提供，不能据 fixture 宣称硬件已接通。

```bash
cd /home/sunrise/east_eleair
# 本地归档验 SHA 后构建这两个包；不会下载或覆盖原安装
bash tank_inspection_system/scripts/build.sh --wireless-only
source tank_inspection_system/scripts/env.sh
# 先核对唯一配置 physic_bev/src/system_websocket/agv_bridge/config/tank_inspection.yaml
# 默认底盘控制关闭；启动仅连接 ROS topic 和本地 WebSocket 9001
ros2 launch agv_bridge tank_inspection.launch.py
```

本机新增依赖现已离线缓存为 32 个 arm64 deb 与 websockets 15.0.1 wheel，SHA/版本见 `offline/dependency_manifest.json`；D405 实测时补齐 image_transport、diagnostic_updater 两个运行库。运行不依赖互联网，构建移除了 GitHub FetchContent。现有 Ubuntu/Humble、Rerun、传感器 SDK 和编译器保持原安装。此缓存提供本机新增依赖，未交付未知硬件的 SDK，也不是裸机系统镜像。双端 WebSocket、断线重连、原速率 DDS、双视频和 watchdog 的原测试使用断网代理及本地 fixture；D405 已另做实际设备验证，物理无线链路与其余设备尚未实测。协议、配置、离线复测及媒体恢复步骤见 [操作文档](docs/collection_and_replay.md)。

上位机验证页面为 [websocket_console/index.html](websocket_console/index.html)，仅由该 HTML 和同目录 `console.js` 构成，不需要 npm、CDN 或互联网。将两个文件一起复制到上位机，双击 HTML，填写 `ws://板端实际IP:9001` 后连接。页面显示六设备消息自检、涡流 raw/mean 独立状态、双路 JPEG、雷达位姿、桥接统计与共享回包；支持视频开关、自动重连和导出脱敏验证记录。底盘命令需人工勾选后才允许发送速度，默认值全零。收到 ROS 发布回包不代表底盘实际执行。

页面已在本机 Firefox 140.0.4 中连接实际安装的桥接节点，以隔离 ROS fixture 验证；不计为物理无线或硬件验收。使用和复测命令见 [操作文档的上位机部分](docs/collection_and_replay.md#上位机-websocket-验证页面)，证据追加在 `final_report.json.closeout.websocket_console`。上位机验证阶段没有开展D405融合；后续独立实验框架现已增加数据/视觉/深度/融合、训练导出与被动ROS，状态与实测见其README和framework_validation.json，真实精度仍未验收。

2026-10-04 D405 实机接通：设备序列号 `352122271190`，固件 5.17.0.10，ROS 驱动 4.57.7 / SDK 2.58.4，USB 3.2。Depth/Infra1/Infra2 均为 848×480、配置 30 FPS；独立 25 秒测量接收约 29.52/29.91/29.60 FPS，存在少量间隔缺口，未宣称无丢帧。Factory CameraInfo、流间外参、真实 12 条消息的 MCAP 写入/关闭/读取/CDR 字节核对、正常停止/重启及 Firefox 深度 JPEG 显示已验证。整体硬件与焊缝算法验收仍未通过。

`env.sh` 通过 `FIELD_REALSENSE_LIB_DIR` 引用已存在的 `third_party/librealsense/build/release/librealsense2.so.2.58`；此目录是当前驱动的必要运行依赖，不能作为临时构建目录清理。没有重建、升级 SDK 或覆盖原安装。当前测试服务仍运行，可在上位机打开 `http://192.168.127.10:8080/`（另一网口 `http://192.168.1.6:8080/`），点击连接；跨机器访问仍需实际局域网可达。启动、停止与契约细节见 [D405 实机验证](docs/collection_and_replay.md#d405-单设备实机验证2026-10-04)。

用户随后要求上位机查看 RGB。当前桥接订阅 `/d405/camera/color/image_raw`，D405 硬件 Color 使用 RGB8 848×480×30；Depth/Infra1/Infra2 的 ROS 发布保留。页面改为 RGB 预览，协议字段 `depth_camera` 保持兼容。唯一源 YAML 已更新，独立安装为 `install_wireless_rgb`；旧 `install_wireless` 保留用于回退。重新 source `env.sh` 后使用同一个 `ros2 launch agv_bridge tank_inspection.launch.py`。RGB 启动配置、回退与环境一致性见现有操作文档；结果集中追加在 `closeout.d405_hardware_validation.rgb_preview`，不覆盖前面的深度实测证据。

后续3Hz显示问题已修复，当前默认独立安装改为 `install_wireless_preview`，上述两份旧安装保留。罐体预览目标15FPS，实际RGB收到/解码由2.59提升到14.94FPS；D405 RGB＋深度融合overlay经真实ROS→WebSocket→Firefox链路，20.2秒实际解码11.43FPS、处理11.98Hz，完整回调P95 91.8ms。两秒显示窗口最低9.77FPS，因此平均超过10Hz，不宣称所有时间窗口或任意现场负载保证10Hz。现场420帧均LOST，未验收真实焊缝精度、物理无线或长期实时性。浏览器Ctrl+F5刷新后分别显示“显示/收到FPS”；旧终端显式设置FIELD_WIRELESS_INSTALL，具体命令见 [显示帧率修复与复测](docs/collection_and_replay.md#显示帧率修复与复测)。证据与旧阶段区别记录在现有final_report.json.closeout.d405_preview_rate_fix及D405框架报告preview_rate_fix。
