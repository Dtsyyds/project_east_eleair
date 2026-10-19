# Topic 契约

权威消息仍在各包 .msg 与 driver 源码；本文件是审计说明，机器可读细节见 ../manifests/topic_contract.yaml。硬件 offered QoS、device frame_id 和时间域未测量的部分明确未知，不写已验证。

|topic|消息|单位/时间|QoS 与限制|
|---|---|---|---|
|`/camera/front/image_raw`|sensor_msgs/msg/Image|pixel encoding per message; mono8/bgr8 according to SDK layout；exposure/device tick software anchored to host ROS|reliable when front_video=true; standalone default best_effort；only driver owns camera; video ROS reliable subscription|
|`/camera/front/camera_info`|sensor_msgs/msg/CameraInfo|pixels; K/P zero uncalibrated；exposure/device tick software anchored to host ROS|reliable when front_video=true; standalone default best_effort；|
|`/camera/front/image_h264`|img_msgs/msg/H26XFrame|H264 bytes; pts/dts ns；codec source ROS header plus codec pts/dts|recorder subscriber reliable；source header is authoritative; packets are not decoded frames|
|`/livox/lidar`|sensor_msgs/msg/PointCloud2 (vehicle xfer_format=0); livox_ros_driver2/msg/CustomMsg for configured alternate mode|x/y/z metres; intensity/point time per Livox fields；Livox driver nanosecond data timestamp|reliable publisher; sync subscriber best_effort；FAST-LIVO2 lidar_type=1 requires CustomMsg; do not silently reinterpret PointCloud2|
|`/livox/imu_raw`|sensor_msgs/msg/Imu|acceleration native g before adapter; gyro SDK angular rate；ImuData.time_stamp ns|reliable driver default；|
|`/livox/imu`|sensor_msgs/msg/Imu|adapter acceleration_scale default9.80665 converts g to m/s²; angular velocity preserved；copied raw stamp, no clock recalibration|see adapter source; verify actual runtime offered QoS；|
|`/d405/camera/depth/image_rect_raw`|sensor_msgs/msg/Image|depth encoding read from each message; 16UC1 typically scale via RealSense depth units, not assumed; IR pixels；device/host/global_time depending frozen startup parameters|device source/config dependent; topic_inventory required；device header must be nonzero/strictly increasing; rates not established by fixture|
|`/d405/camera/infra1/image_rect_raw`|sensor_msgs/msg/Image|depth encoding read from each message; 16UC1 typically scale via RealSense depth units, not assumed; IR pixels；device/host/global_time depending frozen startup parameters|device source/config dependent; topic_inventory required；device header must be nonzero/strictly increasing; rates not established by fixture|
|`/d405/camera/infra2/image_rect_raw`|sensor_msgs/msg/Image|depth encoding read from each message; 16UC1 typically scale via RealSense depth units, not assumed; IR pixels；device/host/global_time depending frozen startup parameters|device source/config dependent; topic_inventory required；device header must be nonzero/strictly increasing; rates not established by fixture|
|`/d405/camera/{depth,infra1,infra2}/camera_info`|sensor_msgs/msg/CameraInfo|device intrinsics per message; rig extrinsics unverified；device stream header|runtime device config；|
|`/eddy/data`|eddy_driver/msg/EddyFrame|20 samples ×8 channels int16 mV; sample-major index=s*8+c; sequence uint32；system host_receive after complete UART frame|best_effort；no device sampling rate field; sync uses actual sequence; 160 values not means|
|`/eddy/channel_mean`|eddy_driver/msg/EddyChannelMean|float64[8] mV; computed over20samples；system host_receive after complete UART frame|best_effort；mean only; no raw/IQ/sequence; sync explicitly uses local arrival index|
|`/eddy/status`|eddy_driver/msg/EddyStatus|protocol/status bytes and diagnostics per message；host ROS status sample time|reliable；last_status_is_error propagated to collection acceptance|
|`/eddy/cmd`|eddy_driver/msg/EddyCommand|device protocol commands；headerless available bag/host time|reliable；no command issued in this task|
|`/inspection/eddy_current/raw`|inspection_interfaces/msg/EddyCurrentFrame|I/Q, sampling_rate_hz, gain/liftoff per legacy msg contract；legacy header stamp; positive required|sync subscriber best_effort；legacy contract preserved; native driver not cast into this richer message|
|`/derived/inspection/fusion_index`|inspection_interfaces/msg/FusionIndex|time errors ns; derived pose SI; u/v/s/n metres only when asset_coordinate_valid；selected anchor header copied; camera/lidar/imu/pose stamps separately recorded|reliable；raw+mean anchor fixtures5→5 each; default anchor legacy ultrasound; no spatial accuracy claim|
|`/vision/front/seam_geometry`|tank_vision_interfaces/msg/SeamGeometry|geometry pixel coordinates and fitted coefficients/status; no calibrated mm claim；original input header stamp|reliable；|
|`/vision/front/seam_state`|tank_vision_interfaces/msg/SeamState|geometry pixel coordinates and fitted coefficients/status; no calibrated mm claim；original input header stamp|reliable；|
|`/vision/front/debug_image`|sensor_msgs/msg/Image|bgr8 pixels；input header|reliable；publish_debug_image default false; heavy diagnostic switches unchanged|
|`/cloud_registered`|sensor_msgs/msg/PointCloud2|metres/radians; algorithm frame not calibrated tank coordinate；algorithm source measurement/pose header|reliable default；mapping collection default false; attach-only profile available; field validation not executed|
|`/path`|nav_msgs/msg/Path|metres/radians; algorithm frame not calibrated tank coordinate；algorithm source measurement/pose header|reliable default；mapping collection default false; attach-only profile available; field validation not executed|
|`/aft_mapped_to_init`|nav_msgs/msg/Odometry|metres/radians; algorithm frame not calibrated tank coordinate；algorithm source measurement/pose header|reliable default；mapping collection default false; attach-only profile available; field validation not executed|
|`/tf_static`|tf2_msgs/msg/TFMessage|metres/quaternion；static transform; exempt from positive monotonic dynamic header rule|reliable；rough TF is not validated extrinsics|
|`/diagnostics`|diagnostic_msgs/msg/DiagnosticArray|diagnostic strings/values；host ROS stamps; multiple publishers interleave; same-source regression counted|reliable；|

动态 Image/CameraInfo/PointCloud/Imu/Eddy/Odometry 要求所有 header 正且严格递增，并统计 zero/negative/missing/duplicate/regression 与间隔。tf_static 为明确静态例外；headerless使用bag/hosttime。没有把全无效header视为单调性通过。

无线桥仅订阅以上已有接口，不更改 topic/消息或设备 QoS。图像、涡流、EMAT、点云订阅 SensorDataQoS best_effort/volatile/depth1，可匹配原 reliable 发布；Odometry/底盘状态 best_effort/depth10。启用控制时新增可配置的 Twist 或 Float64MultiArray 命令发布，reliable/depth10。鱼眼、EMAT、底盘未知真实 topic 必须通过唯一配置提供。双路 JPEG 仅限频预览，原帧 header 在 WebSocket stamp_ns 十进制字符串和 sec/nanosec 中精确保留。完整字段、单位、消息活性边界和命令 ACK 含义见 collection_and_replay.md；机器配置引用在 manifests/topic_contract.yaml.wireless，默认源始终是 agv_bridge/config/tank_inspection.yaml。

回放实际 remap：sensor-only/full-sensor 将 `/livox/lidar` 保持 PointCloud2 类型发布到 `/livox/lidar_points`，将原 g 单位 `/livox/imu_raw` 经已有安装的 C++ adapter 转成 `/livox/imu`（m/s²，gyro rad/s不变，orientation_covariance[0]=-1）。public QoS 仍 reliable/volatile/depth10。原 bag 消息不改写。algorithm-replay 图像/CameraInfo 到 `/replay/camera/front/*`，其他原记录到 `/replay/recorded*`；新增原速率 DDS 断言覆盖这些名字与数值。
