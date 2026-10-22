# Refactor V1 行为登记

正常同步算法、ROI/reference/阈值/DP/IRLS/temporal 和模型配置未调整。冻结图像/连续序列修改前后业务结果 SHA 相同，trace 开关另有一致性检查。

异常和验收行为变化：

1. worker 异常由日志吞掉改为 FAILED 并对 submit/flush/close 可见；统一关闭 deadline 包括排空、底层 writer 收尾和关闭状态文件 I/O；超时保留 worker_alive/metadata_worker_alive/incomplete，不强杀线程。状态文件阻塞时调用方及时获得 FAILED/close_error，I/O 恢复后重算并补写 incomplete。离线 bounded output 必须并发消费；实时 latest output 的旧结果丢弃显式计数。demo 离线逐帧写入，live 每批最多两帧。
2.统计分位数改为最近 2048 帧的精确线性插值；均值/最大值仍为全量在线统计。相机统计同样只保留 2048 条。
3.新视频保留编码 journal 与解码 frame_map；只有实际帧身份能证明对齐才发布最终文件。旧 run 没有证据的严格对齐由过去数量比例通过改为失败，绝不修改旧 sidecar。
4.RRD strict 重新计算选中输入的 SHA，提交前再次检查；全零 SHA、缺失、逃逸、封存不合法、运行中篡改拒绝提交。失败保留 partial；dry-run 执行了哈希时明确 hash_verified=true。
5.动态传感器无有效时间戳不再单调性通过；关闭模块写 disabled，未执行模块写 not_executed。涡流监控错误进入最终验收，历史 performance 固定成功描述移除。
6.旧 EddyCurrentFrame 订阅保留，增加 EddyFrame / EddyChannelMean 时间适配；均值不制造 I/Q 或波形。同步默认入口仍关闭，缺乏坐标标定时 asset_coordinate_valid=false。
7.封存 run 的 stop/finalize 不再重写报告；validate/replay 使用独立分析目录。handoff 从当前 run manifest 解析 RRD 名称，不固定历史批次。
8.安装版 stable 默认配置缺少 guided_search 曾导致 KeyError；不适用的调参映射现在列为 unavailable，不修改算法默认值。
9.本机 setuptools 59.6 的离线打包通过从 pyproject.toml 读取 metadata 的 setup.py 兼容，避免 UNKNOWN 空包；没有升级依赖。

完整逐文件信息、复现和测试路径见 reports/refactor_v1。此前 dirty/untracked 文件原样备份后增量修改，没有提交或恢复 Git 状态。

## 本次自检与无线接入

- 通过本地归档修复 MCAP 插件缺失；保留原 SDK/core，实际读写/CDR、原失败测试、多 bag DDS、RRD 与 handoff 通过。
- 加入 tank_inspection_robot 的被动 ROS 自检、鱼眼/D405 JPEG 预览、Odometry 位姿和默认关闭的显式底盘命令转发。未配置设备不伪造通过，未启动真实硬件。
- 修复 WebSocket 本地响应缺失、无限消息积压、重连生命周期和异常关闭；首个 accept 在处理器注册后创建，端口占用明确返回 2。未知机器人 ID 拒绝；停止仅针对自有 legacy 进程组，移除全局 pkill。
- WebSocket 精确 stamp_ns 使用字符串，避免 JS 53bit 精度丢失；原 ROS/采集格式不变。新底盘响应区分 ROS 发布与硬件 ACK。
- core wheel 直接构建/验源 SHA，替代重复 staging 副本；默认离线验证临时产物清理，日志复用，结果只追加 final_report.json.closeout。

- 回放在原媒体时间轴前有界等待 DDS 发现，修复原速率首条消息丢失；新增 discovery-wait-sec=0 可选择原立即开始行为。canonical shell 只 source 完整 TROS 环境，并保留已选 overlay 的 local_setup。
- 修复 native IMU g 数值被直接改名为 SI topic 的缺陷；复用实际安装的 C++ adapter，通过 private topic 转换并保持 public QoS/header。积压/转换失败对调用方可见，不静默丢结果。
- 同版本 core wheel 采用显式强制重装；隔离目标复现/验证，当前安装未覆盖。无线 invalid/accepted 统计拆开，停止响应也标记只完成 ROS 发布。
