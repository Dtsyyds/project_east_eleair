# D405 profile 与 FAST-LIVO 相机参数记录

采集日期：2026-09-23。设备为 Intel RealSense D405，ROS 节点为
`/camera/camera`。

## 已确认的参数归属

当前节点中 `rgb_camera.color_profile` 显示为 `Parameter not set`。
实际控制当前彩色图像话题
`/camera/camera/color/image_raw` 的参数是：

```text
depth_module.color_profile
```

因此，修改 D405 的实际彩色采集 profile 时应修改该参数，而不是依赖
`rgb_camera.color_profile`。

当前深度流已关闭（`enable_depth:=false`）。`depth_module.depth_profile`
不决定彩色图像，但应使用一个设备支持的值，以避免驱动在启动时报告
profile 无效。

## D405 当前驱动报告的支持 profile

`depth_module.color_profile`：

```text
1280x720x5    1280x720x15   1280x720x30
424x240x5     424x240x15    424x240x30    424x240x60    424x240x90
480x270x5     480x270x15    480x270x30    480x270x60    480x270x90
640x360x5     640x360x15    640x360x30    640x360x60    640x360x90
640x480x5     640x480x15    640x480x30    640x480x60    640x480x90
848x480x5     848x480x15    848x480x30    848x480x60    848x480x90
```

`depth_module.depth_profile`：

```text
1280x720x5    1280x720x15   1280x720x30
256x144x90
424x240x5     424x240x15    424x240x30    424x240x60    424x240x90
480x270x5     480x270x15    480x270x30    480x270x60    480x270x90
640x360x5     640x360x15    640x360x30    640x360x60    640x360x90
640x480x5     640x480x15    640x480x30    640x480x60    640x480x90
848x480x5     848x480x15    848x480x30    848x480x60    848x480x90
```

## 已测得的 848x480 当前相机内参

当前 `/camera/camera/color/camera_info`：

```text
width  = 848
height = 480
K = [437.34460449, 0, 422.19131470,
     0, 436.23373413, 238.20819092,
     0, 0, 1]
D = [-0.05356108397245407,
      0.05783652886748314,
      0.00034853132092393935,
     -0.00042196811409667134,
     -0.019268931820988655]
```

这些数值**仅适用于 848x480 profile**，不能用于 640x360。

## 切换目标：640x360x30

`640x360x30` 已出现在 D405 的支持列表中，因此可用。

在 `src/top_pkg/launch/bringup_sensor.launch.py` 的 RealSense
`launch_arguments` 中应使用：

```python
"enable_depth": "false",
"enable_color": "true",
"depth_module.color_profile": "640x360x30",
"depth_module.depth_profile": "640x360x30",
```

`depth_module.depth_profile` 虽然在深度关闭时不启流，但保留有效值可避免
无效 profile 的启动错误。

不要将 `rgb_camera.color_profile` 作为 D405 当前彩色流 profile 的唯一配置。

## FAST-LIVO 相机模型（必须使用 640x360 实测内参）

切换 profile 后，先单独启动相机并读取：

```bash
ros2 topic echo --once /camera/camera/color/camera_info --field width
ros2 topic echo --once /camera/camera/color/camera_info --field height
ros2 topic echo --once /camera/camera/color/camera_info --field k
ros2 topic echo --once /camera/camera/color/camera_info --field d
```

然后在 `src/FAST-LIVO2/config/d405.yaml` 中使用：

```yaml
camera:
  cam_model: Pinhole
  cam_width: 640
  cam_height: 360
  scale: 1.0
  cam_fx: <K[0]>
  cam_fy: <K[4]>
  cam_cx: <K[2]>
  cam_cy: <K[5]>
  cam_d0: <D[0]>
  cam_d1: <D[1]>
  cam_d2: <D[2]>
  cam_d3: <D[3]>
```

`D[4]` 当前 FAST-LIVO 的 Pinhole 配置未使用。不能使用本文件记录的
848x480 内参，也不应仅按比例缩放旧参数；应以切换后 RealSense 发布的
640x360 `camera_info` 为准。

## 验证

启动完整系统前，确认：

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic echo --once /camera/camera/color/image_raw --field width
ros2 topic echo --once /camera/camera/color/image_raw --field height
```

预期为约 30 Hz、宽 640、高 360。

## 640x360x30 实测结果与最终配置

已实测：

```text
width = 640
height = 360
frequency = 29.996 Hz（约 30 Hz）
K = [328.00845337, 0, 318.64349365,
     0, 327.17529297, 178.65614319,
     0, 0, 1]
D = [-0.05356108397245407,
      0.05783652886748314,
      0.00034853132092393935,
     -0.00042196811409667134,
     -0.019268931820988655]
```

最终 RealSense 启动参数：

```python
"enable_depth": "false",
"enable_color": "true",
"depth_module.color_profile": "640x360x30",
"depth_module.depth_profile": "640x360x30",
```

最终 FAST-LIVO 相机模型：

```yaml
camera:
  cam_model: Pinhole
  cam_width: 640
  cam_height: 360
  scale: 1.0
  cam_fx: 328.00845337
  cam_fy: 327.17529297
  cam_cx: 318.64349365
  cam_cy: 178.65614319
  cam_d0: -0.05356108397245407
  cam_d1: 0.05783652886748314
  cam_d2: 0.00034853132092393935
  cam_d3: -0.00042196811409667134
```

`D[4]` 当前 FAST-LIVO 的 Pinhole 配置未使用。畸变系数无量纲，不随
分辨率或 `scale` 等比例缩放。
