# 整机采集与 WebSocket 操作说明

工程目录：`/media/mcap_data/east_eleair_plus`

## 1. 启动前检查

```bash
cd /media/mcap_data/east_eleair_plus
source tank_inspection_system/scripts/env.sh
set +u
source physic_bev/install_lifecycle/setup.bash
export ROS_DOMAIN_ID=77

lsusb -d 8086:0b5b && rs-enumerate-devices -s
```

必须能够识别 D405。使用真实 ECA 时还应存在 `/dev/eddy`：

```bash
ls -l /dev/eddy
```

## 2. 终端一：启动全部设备并录制 MCAP

```bash
cd /media/mcap_data/east_eleair_plus
source tank_inspection_system/scripts/env.sh
set +u
source physic_bev/install_lifecycle/setup.bash
export ROS_DOMAIN_ID=77

MCAP_OUT="/media/mcap_data/mcap/full_system_$(date +%Y%m%d_%H%M%S)"

ros2 launch inspection_bringup hardware_pipeline.launch.py \
  record:=true output_directory:="$MCAP_OUT" \
  start_eddy:=true start_sync:=true \
  d405_serial_number:=260322276866 \
  d405_rgb_profile:=1280x720x30 \
  d405_depth_profile:=1280x720x30
```

该命令启动 MindVision、鱼眼、D405、MID-360、真实 ECA，并将数据保存为分片 MCAP。

如果没有连接真实 ECA，只将上面的参数改为：

```text
start_eddy:=false start_sync:=false
```

## 3. 终端二：启动 WebSocket

```bash
cd /media/mcap_data/east_eleair_plus
source tank_inspection_system/scripts/env.sh
set +u
source physic_bev/install_lifecycle/setup.bash
export ROS_DOMAIN_ID=77

ss -ltnp '( sport = :9001 )'
ros2 launch agv_bridge tank_inspection.launch.py \
  config:="$PWD/physic_bev/src/system_websocket/agv_bridge/config/tank_inspection.yaml"
```

如果 `ss` 已显示 9001 正在监听，不要再次执行 `ros2 launch`。

上位机连接地址：

```bash
ip -4 addr show dev eth1
```

```text
ws://<eth1的IPv4地址>:9001
```

## 4. 简单检查

```bash
ros2 topic hz /camera/front/image_raw
ros2 topic hz /camera/fisheye/image_raw
ros2 topic hz /d405/camera/color/image_raw
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

## 5. 停止和验证

先在 WebSocket 终端按 `Ctrl+C`，再在采集终端按 `Ctrl+C`，等待 MCAP 正常关闭。不要使用 `kill -9`。

采集终端仍保留 `$MCAP_OUT` 时执行：

```bash
ros2 bag info "$MCAP_OUT"
```

注意：MindVision 4096×3072 BGR8 数据量很大，不建议无人值守长时间录制。只有所有目标 Topic 均有实际消息时，才能视为完整采集。
