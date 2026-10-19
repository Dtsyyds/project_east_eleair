# EMAT 单次串口采集脚本

`emat_serial_once.py` 供 systemd timer、cron 或其他调度程序每次独立启动。调用模式明确分为 `--start` 与 `--acquire`：

1. `--start`：每次启动都发送 `81 01`，仅在启动状态接受单字节 `01`；再发送 `01`，仅在清零状态接受单字节 `02` 清零地址；不读取采样数据。
2. `--acquire`：只发送 `00`，读取并输出 `--expected-data-bytes` 个原始字节；不发送启动或清零命令。

读取状态不根据字节值猜测含义：测量数据中的 `00`、`01`、`02` 都会原样输出。

## Ubuntu 22.04 使用

```bash
sudo apt install python3-serial
chmod +x emat_serial_once.py
./emat_serial_once.py \
  --serial-port /dev/serial/by-id/实际设备路径 \
  --baudrate 实际波特率 \
  --start
```

启动调用在完整收到启动 ACK `01` 和清零 ACK `02` 后，默认输出 `01`；启动失败则输出 `00`（并以退出码 `1` 表示失败）。采集调用才输出一个十六进制数据行，例如数据 `0x00` 输出 `00`，数据 `0x01` 输出 `01`：

```bash
./emat_serial_once.py --serial-port /dev/serial/by-id/实际设备路径 \
  --baudrate 实际波特率 --acquire --expected-data-bytes 1
```

失败时不输出测量数据，诊断信息写入标准错误，退出码为 `1`。

排查串口协议时添加 `--raw-hex-trace`。它将收发字节打印到标准错误，不影响标准输出的数据结果：

```bash
./emat_serial_once.py --serial-port /dev/serial/by-id/实际设备路径 \
  --baudrate 实际波特率 --start --raw-hex-trace
```

若调度程序需要未改动的二进制字节，使用 `--output-format raw`；需要长度和延迟等元数据时，使用 `--output-format json`。

每次启动均自动清零地址；采集模式不发送清零命令：

```bash
./emat_serial_once.py --serial-port /dev/serial/by-id/实际设备路径 \
  --baudrate 实际波特率 --start --output-format json
```

波特率、数据位、校验、停止位、流控、ACK/读取/写入超时、命令间等待以及返回长度都可通过命令行配置。当前实测的默认命令间等待为 **800 ms**；仍可用 `--inter-command-delay-ms` 覆盖，后续应按实机稳定性测试继续优化。
