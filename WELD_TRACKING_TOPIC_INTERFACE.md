# 焊缝视觉偏差 ROS 2 通信格式

话题名：`/weld_tracking/error`  
消息类型（拟新增）：`duct_robot_msgs/msg/WeldTrackingError`

## 消息定义

文件名：`WeldTrackingError.msg`

```text
# 图像采集时间；frame_id 为相机光学坐标系名称
std_msgs/Header header

# 图像帧序号，单次运行内递增
uint64 frame_seq

# 横向误差和方向误差是否都有效；false 时忽略两项误差
bool valid

# 归一化横向误差，无量纲：焊缝偏左为负，偏右为正，0 表示对齐
float64 lateral_error_norm

# 图像平面方向误差，单位 rad：焊缝走向相对参考前进方向偏左为正，偏右为负
# 0 表示方向平行；焊缝方向统一指向机器人前方，避免正反向产生约 pi 的跳变
float64 heading_error_rad

# 两项误差共同的识别置信度，范围 0～1
float32 confidence
```

## 偏差约定

横向误差在固定图像参考行测量：

```text
lateral_error_norm = (焊缝中心x - 目标参考中线x) / (处理图像宽度 / 2.0)
```

例如图像宽 640、参考中线 x=320、焊缝中心 x=288，则 `lateral_error_norm=-0.10`，表示偏左 32 像素。

方向误差为参考行附近焊缝中心线的局部走向，相对目标参考中线前进方向的有符号夹角。焊缝走向向左偏 3° 时，`heading_error_rad=+0.05236`；向右偏 3° 时为 `-0.05236`。角度换算为 `rad = deg × pi / 180`。

双方固定同一图像参考行、方向拟合区域和目标参考中线，统一图像方向，使图像左侧对应机器人前进方向的左侧。焊缝中心、参考中线、方向角和图像宽度均基于同一处理图像。目标参考中线按车体或探头工作中心确定。

本版横向误差为归一化图像量，方向误差为图像平面夹角，均不是经过相机标定换算的壁面物理误差。若改用米制横向误差或车体平面方向角，须另行约定字段和坐标系。

## 发布示例

```yaml
header:
  stamp:
    sec: 1793664000
    nanosec: 123456789
  frame_id: camera_optical_frame
frame_seq: 123
valid: true
lateral_error_norm: -0.10
heading_error_rad: 0.05236
confidence: 0.95
```

该示例表示焊缝中心在参考中线左侧，局部走向也相对参考前进方向向左偏约 3°。

时间戳填写对应图像的实际采集时间。每个完成识别的图像帧发布一次；仅当两项误差均可可靠计算、数值均为有限值时，才发布 `valid: true`。

焊缝丢失或任一误差不可可靠计算时发布：

```yaml
header:
  stamp:
    sec: 1793664000
    nanosec: 173456789
  frame_id: camera_optical_frame
frame_seq: 124
valid: false
lateral_error_norm: 0.0
heading_error_rad: 0.0
confidence: 0.0
```

无效消息中的两个零值仅为占位值，不表示已经对齐。

QoS：发布端与订阅端均使用 `KeepLast(1)`、`BestEffort`、`Volatile`。
