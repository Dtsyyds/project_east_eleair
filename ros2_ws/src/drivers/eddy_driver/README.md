# eddy_driver

STM32 `eddy_stm32` V1.0 通信协议的 ROS 2 串口驱动。

## 串口与报文协议

驱动使用 USART1 的串口链路：4 000 000 bit/s、8N1、无流控。

AD7606 采样帧固定为 324 字节：

```text
55 AA | 320 字节有效载荷 | 7E FE
```

有效载荷内容：

```text
20 个采样点 × 8 通道 × `int16` 小端序，单位为 mV
```

控制命令与响应固定为 7 字节：

```text
AA | CMD | uint32 小端序数据 | 55
```

命令列表：

- `0x01`：设置幅度；成功响应为 `0x81`
- `0x02`：设置频率（Hz）；成功响应为 `0x82`
- `0x11`：查询幅度；响应仍为 `0x11`
- `0x12`：查询频率；响应仍为 `0x12`
- `0xE0`：错误响应；数据四字节依次为原命令、失败阶段、HAL 状态和附加信息

幅度命令的低 16 位有效，固件将实际值限幅到 `1–800`；频率为完整
`uint32` Hz，固件将实际值限幅到 `5–216000000` Hz。驱动会原样发送 ROS
命令，并只以 STM32 读回响应更新当前幅度和频率。

## 架构

```text
STM32 USB 串口
      |
      v
 EddyDriver 接收线程
      |
      v
 有界 FIFO
      |
      v
 EddyNode 发布线程
      |
      +----> /eddy/data   (EddyFrame，SensorDataQoS，每个完整 ADC 帧对应一条 ROS 消息)
      +----> /eddy/channel_mean (EddyChannelMean，仅时间戳与 8 通道均值)
      +----> /eddy/status (EddyStatus，低速率)
      <----  /eddy/cmd    (EddyCommand)
```

`/eddy/data` 有意不设置固定的 `publish_rate`。每收到一个完整的 ADC 帧，就立即从 FIFO 中取出并尽快发布。

## 编译

将本文件夹放入 ROS 2 工作空间的 `src/` 目录：

```bash
cd ~/my_robot_ws
colcon build --packages-select eddy_driver --symlink-install
source install/setup.bash
```

## 固定设备路径 `/dev/eddy`

本包默认使用 `/dev/eddy`，而不是会因插拔顺序改变的 `/dev/ttyACM0`。随包提供的
udev 规则已锁定当前 Eddy 设备的 USB 序列号 `5B91074919`。首次安装时执行：

```bash
sudo install -m 644 $(ros2 pkg prefix eddy_driver)/share/eddy_driver/udev/99-eddy.rules \
  /etc/udev/rules.d/99-eddy.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
ls -l /dev/eddy
```

若最后一条命令未显示 `/dev/eddy`，请拔插 Eddy 设备后再检查。规则会把设备授予
`dialout` 组；首次配置用户组时运行 `sudo usermod -aG dialout $USER`，随后重新登录。

## 运行

```bash
ros2 launch eddy_driver eddy_driver.launch.py
```

或者：

```bash
ros2 run eddy_driver eddy_driver_node --ros-args --params-file \
  $(ros2 pkg prefix eddy_driver)/share/eddy_driver/config/eddy.yaml
```

## 话题

### `/eddy/data`

类型：

```text
eddy_driver/msg/EddyFrame
```

`data[160]` 的展平顺序：

```text
sample0_ch0 ... sample0_ch7,
sample1_ch0 ... sample1_ch7,
...
sample19_ch0 ... sample19_ch7
```

`channel_mean[8]` 存放各通道在 20 个采样点上的均值。

消息的 `header.stamp` 是主机在解析完一帧完整 UART ADC 数据后立即记录的系统时间，并非 STM32 ADC 的硬件时间戳。

### `/eddy/channel_mean`

类型：

```text
eddy_driver/msg/EddyChannelMean
```

仅包含与 `/eddy/data` 相同的 `header` 和 `float64[8] channel_mean`。车辆 MCAP
采集使用此话题，以保留同步时间戳和 8 路均值，同时不写入 `data[160]`、激励设置、
序号、通道/采样数量或驱动丢帧计数。

### `/eddy/status`

类型：

```text
eddy_driver/msg/EddyStatus
```

包含当前幅值/频率、最近一次状态响应、已接收帧数、FIFO 丢帧数以及队列深度。
收到 `0xE0` 时，`last_status_is_error` 与 `error_command`、`error_stage`、
`error_hal_status`、`error_detail` 会给出手册定义的四个错误字节。

### `/eddy/cmd`

类型：

```text
eddy_driver/msg/EddyCommand
```

示例：

将幅值设置为 100：

```bash
ros2 topic pub --once /eddy/cmd eddy_driver/msg/EddyCommand \
  "{command: 1, value: 100}"
```

将频率设置为 450000 Hz：

```bash
ros2 topic pub --once /eddy/cmd eddy_driver/msg/EddyCommand \
  "{command: 2, value: 450000}"
```

查询幅值：

```bash
ros2 topic pub --once /eddy/cmd eddy_driver/msg/EddyCommand \
  "{command: 17, value: 0}"
```

查询频率：

```bash
ros2 topic pub --once /eddy/cmd eddy_driver/msg/EddyCommand \
  "{command: 18, value: 0}"
```

## 常用检查命令

```bash
ros2 topic hz /eddy/data
ros2 topic echo /eddy/status
ros2 topic echo /eddy/data --once
```

同步录制机器人数据与涡流数据：

```bash
ros2 bag record /eddy/data /eddy/status /joint_states /tf /tf_static
```

## 重要同步说明

在当前协议下，STM32 的 ADC 帧不含硬件时间戳，也不含帧序号，因此驱动只能在主机侧自行分配序号和接收时间戳。对于高速扫描和精确的空间对齐，后续 STM32 协议版本应在每帧中加入 STM32 序列号和 ADC 采集时间戳。
