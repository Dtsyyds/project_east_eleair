# 2026-11-09 工业相机克隆部署修复

## 版本核对

已通过 GitHub 查询及 `git fetch origin` 核对远程 `Dtsyyds/project_east_eleair`。
11 月 7 日最后提交为 `5d67a3101b242c259d611d68eaf351e6e4035022`，
本地及远程 main 均为 `0c7231e91ad064842ce3bbf9dafcf0e3572d437b`。
两者树差异只有 `git仓库管理.md`，业务代码已包含 11 月 7 日版本。
本次没有修改上位机布局；该远程版本仍为三个视频窗口，不能据此恢复未知的四窗口版本。

## 故障和修复

原日志 `.runtime/s100p/web_console/hardware.log` 显示工业相机驱动启动时报
`MindVision SDK root not found` 并退出。Python 绑定存在，但
`third_party/mindvision/2.1.0.50/lib/arm64/libMVSDK.so` 缺失；该目录被 Git 忽略。

从本机 `/media/mcap_data/east_eleair_plus/MindVisionSDK_v2.1.0.50/sdk` 恢复同版本库。
来源与工程中的 Python 绑定 SHA256 完全一致：
`fe378052741186001d54e90e474b8dd63aac065262b5d178b80a42cb7a12dab9`。
恢复的库 SHA256：`f53bde08472b4fe307e7b55a5a7206b5064780182e4ad72ad2447a4b42a2a281`。
动态库依赖全部解析成功，SDK 回报版本 `2.1.0.50`。

新增 `deploy/restore_mindvision_sdk.py`，校验绑定、ELF 架构和复制校验和，
拒绝覆盖不同的现有库，允许同一来源重复执行。驱动缺库错误附恢复指引。
已重新构建本机 `ros2_ws/install` 中的 `mindvision_camera_driver`。

## 验证

- 恢复脚本及相机 QoS/生命周期回归共 8 项通过。
- 真机序列号 `046041920527`，1920×1200 BGR8，沿用已保存的 120000 μs 曝光。
- 独立 ROS domain 177、WebSocket 19003，仅启动工业相机与桥接；没有启动底盘。
- 最终 ROS 验证：100 帧图像、100 帧 CameraInfo、100 对匹配时间戳，错误列表为空，8.287 FPS。
- 最终 WebSocket 验证：100 帧 JPEG 解码成功，640×400，8.256 FPS，设备状态 `verified / streaming`。
- 最终驱动累计 100 帧时 `timeout_count=0`、`capture_error_count=0`。
- 最终观测区间 `preview_dropped` 为 13→13；连接前已有预览被覆盖，连接后该计数未增长。
- 测试结束后正常停止本次相机与桥接。

最终证据：`.runtime/s100p/mindvision_fix_validation_3/` 下的 `camera.log`、
`bridge.log`、`topic.log`、`websocket.log`、`result.json`。
早期失败尝试保留在 `mindvision_fix_validation/` 和 `mindvision_fix_validation_2/`：
一次测试桥接 CLI 端口类型错误；另一次图像通过，但自检请求发生在预热前，结果为未验证。
最终测试等待相机开始发布后再请求自检，没有改动生产设备在线判定。

SDK 初始化期间仍打印 USB 访问/传输警告；第二次尝试累计 4 次 timeout。
本次证明缺库故障已修复及短时真实图像回传成功，不代表整机并行负载、长时间 USB 稳定性
或远程浏览器无线链路已验收。曝光 120 ms 下约 8.3 FPS 符合现存参数，没有通过调整曝光提高帧率。
