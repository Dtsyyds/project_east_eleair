# 2026-11-02 D405 双序列号兼容

三个运行配置（system、cameras、d405_experiment）、hardware_pipeline 默认参数和
D405 生命周期验证脚本统一允许 SDK 序列号 `260322276866,352122271190`。
选择限定为 D405 型号，保留单序列号固定设备的用法。

RealSense 工程驱动将允许列表与当前设备序列号分开保存，断线重连时不将允许范围
缩为上次连接的设备。枚举时选择第一台匹配设备；同时接入两台仍只采集一台。
允许列表中的每个序列号独立持有进程锁，兼容单序列号启动的占用保护；退出时释放。

验证结果：

- realsense2_camera、inspection_bringup 构建成功，安装于本工程 `.runtime/s100p/install`。
- Python 配置与 bringup 检查：21 passed；C++ 序列号选择测试：5 passed。
- 按 D405 专用配置启动安装后的硬件 launch，未覆盖为单序列号，实际自动选择
  `352122271190`，固件 5.17.0.10，SDK USB 类型 3.2。
- ROS 域 178 下观察 30 秒，640×480×15 四路图像分别为彩色 440、深度 441、
  左红外 442、右红外 442 帧，约 14.86–14.93 Hz，最大间隔约 0.134 秒。
  四路连续性、抽样内容和时间戳检查通过。
- 针对两个序列号分别启动第二个驱动进程，均在打开设备前因占用锁拒绝；测试进程
  正常退出，两个锁均释放。未留下采集服务，未录制图像。

本轮物理接入的是 `352122271190`。`260322276866` 的匹配规则已通过自动化测试，
本轮未实测两台物理相机互换；之前的 USB 断线、降速和长期稳定性问题不因此关闭。

证据：`.runtime/s100p/validation/20261102_d405_serial_compat/` 中的 `summary.json`、
`launch.log`、`streams.json`、`conflict_<serial>.log`。C++ 结果位于
`.runtime/s100p/build/realsense2_camera/test_results/realsense2_camera/gtest_serial_selection.gtest.xml`。
