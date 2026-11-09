# EMAT 单次串口采集脚本

`emat_serial_once.py` 供 systemd timer、cron 或其他调度程序每次独立启动。调用模式明确分为 `--start` 与 `--acquire`：

1. `--start`：每次启动都发送 `81 01`，仅在启动状态接受单字节 `01`；再发送 `01`，仅在清零状态接受单字节 `02` 清零地址；不读取采样数据。
2. `--acquire`：一次运行中固定执行 8192 次采集，采集启动间隔固定为 10 ms（目标频率 100 Hz）。每次发送 `00` 并读取 `--expected-data-bytes` 个原始字节；全部接收成功后，将数据转换为十六进制并保存到文本文件；不发送启动或清零命令。

读取状态不根据字节值猜测含义：测量数据中的 `00`、`01`、`02` 都会原样输出。

## Ubuntu 22.04 使用

```bash
sudo apt install python3-serial
chmod +x emat_serial_once.py
./emat_serial_once.py \
--serial-port /dev/ttyUSB0 \
--baudrate 115200 \
--start
```

启动调用在完整收到启动 ACK `01` 和清零 ACK `02` 后，默认输出 `01`；启动失败则输出 `00`（并以退出码 `1` 表示失败）。`--start` 不会创建或写入数据文件。

启动成功后，只需执行一次采集命令。脚本会在同一个串口连接中以 10 ms 间隔自动发送 8192 次读取命令，正常总采集时间约为 81.92 秒。全部采集成功后，数据会保存到当前目录的 `emat_data.txt`：

```bash
./emat_serial_once.py --serial-port /dev/serial/by-id/实际设备路径 \
  --baudrate 实际波特率 --acquire --expected-data-bytes 1
```

文件中每次接收到的数据之间插入一个空格，不换行，文件末尾也没有空格。例如前三次分别收到 `00`、`01`、`ff`，文件开头为：

```text
00 01 ff
```

如需指定文件名或保存位置，传入 `--output-file`：

```bash
./emat_serial_once.py --serial-port /dev/serial/by-id/实际设备路径 \
  --baudrate 实际波特率 --acquire --expected-data-bytes 1 \
  --output-file ./采集数据.txt
```

相对路径以执行命令时的当前目录为准，目标文件的父目录必须已经存在。每次成功运行 `--acquire` 都会覆盖目标文件，使文件只包含本次 8192 次采集结果。采集途中任何一次失败或超时，都不会写入本次数据；诊断信息写入标准错误，退出码为 `1`。

10 ms 是相邻两次读取命令开始时间之间的间隔，串口收发耗时包含在这 10 ms 内。如果一次串口事务本身超过 10 ms，下一次会在事务完成后立即开始，程序不会并发发送命令。普通 Linux 不是实时系统，因此实际时刻可能存在少量调度抖动；启用 `--raw-hex-trace` 还会增加终端输出开销。

排查串口协议时添加 `--raw-hex-trace`。它将收发字节打印到标准错误，不影响标准输出的数据结果：

```bash
./emat_serial_once.py --serial-port /dev/serial/by-id/实际设备路径 \
  --baudrate 实际波特率 --start --raw-hex-trace
```

若调度程序需要未改动的二进制字节，使用 `--output-format raw`；需要总长度和总耗时等元数据时，使用 `--output-format json`。无论终端输出格式如何，数据文件始终是以单个空格分隔每次接收结果、无换行的十六进制文本。

每次启动均自动清零地址；采集模式不发送清零命令：

```bash
./emat_serial_once.py --serial-port /dev/serial/by-id/实际设备路径 \
  --baudrate 实际波特率 --start --output-format json
```

波特率、数据位、校验、停止位、流控、ACK/读取/写入超时、命令间等待以及返回长度都可通过命令行配置。当前实测的默认命令间等待为 **800 ms**；仍可用 `--inter-command-delay-ms` 覆盖，后续应按实机稳定性测试继续优化。

./emat_serial_once.py \
--serial-port /dev/ttyUSB0 \
--baudrate 115200 \
--acquire \
--expected-data-bytes 1 \
--output-file ./采集数据.txt
