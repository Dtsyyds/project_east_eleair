# MindVision ROS 2 camera driver

This driver publishes the MindVision ISP output from both supported sensor
types. It detects `capability.sIspCapacity.bMonoSensor` at startup:

- MV-SUF1205GM: `MONO8`, `mono8`, one channel.
- MV-SUF1205GC: `BGR8`, `bgr8`, three channels.
- MV-SUS1207GC-T1V-C: Bayer GB input processed once by the SDK ISP to
  `BGR8`, published as ROS `bgr8`, three channels.

The driver does not use a color/mono configuration override. A mismatch is a
startup error rather than a silent conversion.

## Topics

The private topics are:

- `~/image_raw`
- `~/camera_info`

For the supplied launch, `namespace:=camera` and `node_name:=front` produce:

- `/camera/front/image_raw`
- `/camera/front/camera_info`
- `/camera/front/preview/image_raw` (bounded 640x480 BGR8 by the SUS profile)

The image and CameraInfo have the same timestamp and frame ID. The
CameraInfo is deliberately uncalibrated: `K[0] == 0`, and no fabricated
intrinsics or rectification are loaded. The image topic is `image_raw`, never
`image_rect_raw`.

Existing `/camera/camera/depth/*` references elsewhere in this workspace are
legacy D405/bringup-chain topics. They are not silently redirected by this
driver; a future integration must use an explicit launch remap.

## GC S100P profile

The installed profile is `config/mindvision_suf1205gc_s100p.yaml`. The serial
number is a string with the `sn_` prefix to avoid YAML numeric parsing:

```yaml
serial_number: "sn_046081820650"
exposure_mode: auto
ae_min_exposure_us: 100.0
ae_max_exposure_us: 50000.0
ae_min_analog_gain_raw: 2
ae_max_analog_gain_raw: 16
warmup_frames: 60
```

Run it with:

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source install/setup.bash
ros2 launch mindvision_camera_driver mindvision_camera.launch.py
```

The standalone launch defaults to `mindvision_sus1207gc_t1v_c.yaml`. Select
the retained SUF profile explicitly with:

```bash
ros2 launch mindvision_camera_driver mindvision_camera.launch.py \
  config_file:="$(ros2 pkg prefix mindvision_camera_driver)/share/mindvision_camera_driver/config/mindvision_suf1205gc_s100p.yaml"
```

## SUS1207GC / SDK 2.1.0.50

SDK selection order is `sdk_root`, `MINDVISION_SDK_ROOT`, workspace-relative
`MindVisionSDK_v2.1.0.50/sdk`, then legacy `/opt`/`/usr/local` fallbacks. The
driver loads the selected native library by absolute path, pins the official
Python binding to that handle, and reports both path and SDK version in logs
and `/diagnostics`. It never overwrites a system SDK.

The SUS profile selects serial `046041920527`, verifies model
`MV-SUS1207GC`, selects the SDK-reported 4096x3072 preset, and applies the
documented reproducible starting point: 70877 us, 4.0x analog gain, AE target
70, RGB gains 131/110/147 and saturation 100. Unsupported auto/one-shot white
balance requests fail explicitly based on `CameraGetCapability`; they are not
silently accepted. Gamma, contrast, frame-speed preset and color-temperature
index use `-1` to retain the SDK value.

Full BGR8 is about 37.7 MB/frame. The full topic is intended for algorithms
and bounded MCAP recording. WebSocket subscribes to the separate 640x480
preview topic, then applies its existing JPEG rate limit and backpressure
counters. Production MCAP configuration rotates at 10 GiB/900 s, but at full
resolution the size limit is reached first; use short raw bursts unless the
storage budget has been explicitly approved.

The verified GC baseline is 4096x3072 BGR8 with the constrained automatic
exposure above. It reached approximately 49997 us, raw analog gain 16
(the SDK's raw unit, not dB), and approximately 19.95 device-timestamp FPS
with 2400 frames, zero timeouts and zero capture errors.

The driver prints all SDK-supported resolution indices, dimensions and
descriptions before selecting the exact requested 4096x3072 profile. Any
future 1920x1080 or 1920x1200 profile can be selected only if the SDK reports
it exactly.

## QoS and timing

Image and CameraInfo use bounded `KEEP_LAST`, `VOLATILE` QoS with default
`BEST_EFFORT` and depth 1. The depth parameter is restricted to 1 through 4;
there is no unbounded internal queue. This avoids buffering ten approximately
37.7 MB BGR8 frames at full resolution.

The message timestamp is:

```text
timestamp_source=software_anchored_device_tick
hardware_synchronized=false
tick_period_ns=100000
```

The SDK's 32-bit 0.1 ms counter is expanded across normal wraparound. Small
backward jumps are rejected, and the driver logs raw tick, device-timestamp
FPS, host receive FPS, timeout count and capture-error count. This is not
hardware synchronization with Livox, IMU or eddy-current hardware, and this
stage does not modify FusionIndex.

The MCAP topic configuration includes `/camera/front/image_raw` and
`/camera/front/camera_info`. Keep `CameraInfo.K[0] == 0` until a real
calibration is supplied; the driver does not invent intrinsics.
