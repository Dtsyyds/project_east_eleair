# 2026-11-09 原整机入口 camera 退出修复

用户运行 `bash system/run_robot.sh`，会话 `20261109_213409` 的 camera 返回 2。
`camera.log` 显示退出来自 operations 子进程；其日志实际为
`Package 'inspection_operations' not found`，并非工业相机 SDK 再次缺失。

## 原因与修改

`deploy/env.sh` 只加载 `.runtime/s100p/install/local_setup.bash`，本机实际安装
在 `ros2_ws/install`。上一轮独立测试手动 source 了后者，未覆盖干净终端的原入口。

- 环境脚本优先使用 `.runtime/<platform>/install`；该安装不存在时，加载本工程
  `ros2_ws/install`。`TANK_INSTALL_ROOT` 及兼容变量统一指向所选路径。
- 依赖诊断使用实际安装路径。
- 整机启动前独立检查相机子进程环境中的 inspection_bringup、inspection_operations、
  agv_bridge 及 operations Python 导入，缺包时在启动设备前返回明确错误。

## 验证

- `env -i HOME=/home/sunrise PATH=/usr/local/bin:/usr/bin:/bin bash system/run_robot.sh --check` 通过。
- 干净环境可以定位和导入本机 `inspection_operations`。
- 环境安装目录选择、优先级、启动方案及缺包拦截回归：11 项通过。
- 使用原入口生成的默认 all 配置与相同相机后台命令，在干净环境下真实运行 35 秒，
  operations / hardware / bridge 均未触发 supervisor 异常退出。
- 工业相机 WebSocket 100 帧全部解码，8.304 FPS，状态 `verified / streaming`。
  驱动累计 100 帧时 timeout_count=0、capture_error_count=0。
- 测试结束正常退出，camera 后台退出码 0。

证据目录：`.runtime/s100p/validation/camera_entry_1109/`。
`before_operations.log` 保留缺包错误；`supervisor.log`、`operations.log`、
`hardware.log`、`bridge.log`、`websocket.log` 和 `result.json` 为本次验证结果。

发现已有底盘驱动 PID 18750（21:05 启动、父进程 1）仍在运行，因此没有重复启动底盘，
也没有停止该既有进程。验证没有开启采集或运动。整机入口预检通过不代表底盘串口无冲突。
D405 在本轮出现 IR stream start failure；本轮修复不构成所有传感器的稳定性验收。

使用方式不变：重新执行 `bash system/run_robot.sh`，不必先手动 source ROS 安装。
