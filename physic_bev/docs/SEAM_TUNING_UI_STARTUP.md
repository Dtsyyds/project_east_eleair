# MindVision + StableSeam + 实时调参 UI 启动手册

适用设备：MindVision SUF1205GC

相机序列号：`046081820650`

分辨率：`4096 x 3072`

输入模式：`ros_topic`

本手册启动三部分：

1. MindVision ROS 相机驱动；
2. StableSeam 焊缝巡迹节点；
3. StableSeam 实时调参 UI。

三个终端必须分别执行环境初始化。不要同时启动 `stable_seam_direct` 或第二个 MindVision 相机拥有者。

## 终端一：启动 MindVision ROS 相机驱动

打开第一个终端，执行：

```bash
cd /home/sunrise/east_eleair/physic_bev

unset PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/sunrise/east_eleair/physic_bev/install/setup.bash

export MINDVISION_SDK_ROOT=/home/sunrise/east_eleair/MindVisionlinuxSDK
export PYTHONPATH="/home/sunrise/east_eleair/physic_bev/src/mindvision_camera_driver${PYTHONPATH:+:$PYTHONPATH}"

ros2 run mindvision_camera_driver mindvision_camera_node --ros-args \
  -r __ns:=/camera \
  -r __node:=front \
  -p sdk_root:=/home/sunrise/east_eleair/MindVisionlinuxSDK \
  -p serial_number:=sn_046081820650 \
  -p resolution_index:=-1 \
  -p width:=4096 \
  -p height:=3072 \
  -p exposure_mode:=auto \
  -p exposure_us:=0.0 \
  -p ae_min_exposure_us:=100.0 \
  -p ae_max_exposure_us:=50000.0 \
  -p ae_min_analog_gain_raw:=2 \
  -p ae_max_analog_gain_raw:=16 \
  -p warmup_frames:=20 \
  -p publish_rate_hz:=10.0 \
  -p timeout_ms:=1000
```

保持该终端运行。正常日志应包含：

```text
timeout_count=0
capture_error_count=0
host_receive_fps=10.000
```

第二个终端检查相机话题：

```bash
source /opt/ros/humble/setup.bash

ros2 topic list | grep -E '^/camera/front/(image_raw|camera_info)$'
ros2 topic hz /camera/front/image_raw
ros2 topic echo /camera/front/camera_info --once
```

## 终端二：启动 StableSeam 焊缝节点

打开第二个终端，执行：

```bash
cd /home/sunrise/east_eleair

unset PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash

export PYTHONPATH="/home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/src/tank_vision_ros:/home/sunrise/east_eleair/tank_vision_inspection${PYTHONPATH:+:$PYTHONPATH}"

python3 -m tank_vision_ros.stable_seam_node --ros-args \
  -r __node:=stable_seam_node \
  -p input_mode:=ros_topic \
  -p image_topic:=/camera/front/image_raw \
  -p camera_info_topic:=/camera/front/camera_info \
  -p config_file:=/home/sunrise/east_eleair/tank_vision_inspection/configs/s100p_guided_anchor_seam_4096x3072.yaml \
  -p reference_file:=/home/sunrise/east_eleair/tank_vision_inspection/configs/seam_geometry_reference_s100p_live_v1.json \
  -p publish_debug_image:=true \
  -p auto_save_failures:=false \
  -p live_session_output_root:=/media/mcap_data/runs/tuning_ui_live \
  -p snapshot_output_dir:=/media/mcap_data/runs/tuning_ui_failure_snapshots
```

正常日志应包含：

```text
snapshot service ready: /stable_seam_node/save_snapshot
```

并持续发布：

```text
/vision/front/seam_geometry
/vision/front/seam_state
/vision/front/debug_image
```

`CameraInfo is present but uncalibrated (K/P are zero); pixel tracking only` 是当前预期警告，表示当前只做像素级巡迹，不进行毫米换算或物理投影。

## 终端三：启动实时调参 UI

打开第三个终端，执行：

```bash
cd /home/sunrise/east_eleair

unset PYTHONPATH
source /opt/ros/humble/setup.bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash

export PYTHONPATH="/home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/src/tank_vision_ros:/home/sunrise/east_eleair/tank_vision_inspection${PYTHONPATH:+:$PYTHONPATH}"

python3 /home/sunrise/east_eleair/physic_bev/scripts/seam_tuning_ui.py --config /home/sunrise/east_eleair/tank_vision_inspection/configs/s100p_guided_anchor_seam_4096x3072.yaml --node-name stable_seam_node
```

UI 启动后应显示：

- 当前 debug 图像；
- `detected`、`state`、`coverage`、`residual`、`processing_ms`；
- 各运行时参数滑块；
- 每个参数当前帧的 `OK`、`FAIL` 或 `N/A` 状态。

调参 UI 只修改当前运行节点的 `tune_*` 参数，不写回 YAML，也不修改 reference 文件。关闭 UI 后，节点恢复为启动时的运行时参数；重新启动节点后恢复配置文件默认值。

## 调参状态说明

- 绿色 `OK`：当前帧满足该门槛；
- 红色 `FAIL`：当前帧不满足，并显示失败原因或观测值；
- 灰色 `N/A`：当前 `SeamGeometry` 没有直接发布该参数的观测值，不作推断。

当前消息可以直接判断覆盖率、有效跨度、拟合残差以及由 `failure_reasons` 明确报告的点数、inlier、连续缺失和路径置信度失败。

边缘响应、边缘百分位、边缘间距和 guided search 半径属于处理过程参数，当前消息没有逐帧观测值，显示 `N/A` 是有意的。

## 调参服务检查

如果 UI 提示参数服务未连接，在任一已加载相同 ROS 环境的终端执行：

```bash
source /opt/ros/humble/setup.bash
ros2 service list | grep /stable_seam_node/set_parameters
```

应看到：

```text
/stable_seam_node/set_parameters
```

节点终端收到调参后会出现类似日志：

```text
runtime tuning updated: minimum_coverage=...
```

## 正常停止顺序

按以下顺序停止：

1. 关闭调参 UI；
2. 在 StableSeam 终端按 `Ctrl+C`；
3. 在 MindVision 驱动终端按 `Ctrl+C`。

不要使用 `kill -9`，不要同时启动第二个相机节点。

## 常见问题

### `ament_index_python` 找不到

当前终端没有加载 ROS 环境。重新执行：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/install_s100p_humble/setup.bash
```

### `tank_vision.failure_event_recorder` 找不到

检查 `PYTHONPATH` 必须追加，而不是覆盖：

```bash
export PYTHONPATH="/home/sunrise/east_eleair/tank_vision_inspection/ros2_ws/src/tank_vision_ros:/home/sunrise/east_eleair/tank_vision_inspection${PYTHONPATH:+:$PYTHONPATH}"
```

### UI 没有 debug 图像

确认 StableSeam 节点使用：

```text
-p publish_debug_image:=true
```

并检查：

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /vision/front/debug_image
```

调参 UI 已使用传感器数据 QoS，且会将图像自动缩放到窗口区域。

