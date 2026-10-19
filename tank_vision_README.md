# visoin_track

当前工作区的二维视觉检测 MVP 位于 [`tank_vision_inspection/`](tank_vision_inspection/README.md)。

## 验证采集

`
python3 scripts/test_mindvision_camera.py --config configs/camera_mv_suf1205gm.yaml --sdk-root '/home/sunrise/east_eleair/MindVisionlinuxSDK' --frames 30
`

## 稳定性测试

`
python3 scripts/test_mindvision_camera.py --config configs/camera_mv_suf1205gm.yaml --sdk-root '/home/sunrise/east_eleair/MindVisionlinuxSDK' --frames 150 --timeout-ms 1500
`
### 输出结果

CtiPath: /usr/lib/fg_cxp.cti is error or system not find
CXP system count:0
DetectMSDevice....................
MSCamera Begin link......................................
user control fd open failed
BackWorkerStop
begin destory msdevice
end destory msdevice
End DetectMSDevice
{
  "camera_name": "MV-SUF1205GM#0",
  "serial_number": "043110120081",
  "port_type": "USB3.0",
  "frames": 150,
  "sensor_mono": true,
  "frame_formats": [
    [
      4096,
      3072,
      12582912,
      17301505
    ]
  ],
  "source_read_ms": {
    "mean": 281.852,
    "p95": 299.333,
    "max": 588.445
  },
  "device_timestamp_0p1ms_first_last": [
    161656040,
    162073676
  ],
  "exposure_us_first_last": [
    10000,
    99994
  ],
  "analog_gain_first_last": [
    1.0,
    8.0
  ]
}

## 验证实际图像

`
find '/home/dts/下载/linuxSDK_V2.1.0.49202602041120' -maxdepth 5 -type f \
 \( -iname '*demo*.py' -o -iname '*sample*.py' -o -iname '*grab*.py' -o -iname '*capture*.py' \) \
 | sort

(base) dts@dts:~$ cd '/home/sunrise/east_eleair/MindVisionlinuxSDK/demo/python_demo'
(base) dts@dts:~/下载/linuxSDK_V2.1.0.49202602041120/demo/python_demo$
 python3 cv_grab.py 

`

## 支持官方预设分辨率索引和固定曝光，可准确测试 USB 2.0 下的帧率。先测 1920×1200，固定曝光 30 ms

`
python3 scripts/test_mindvision_camera.py --config configs/camera_mv_suf1205gm.yaml --sdk-root '/home/sunrise/east_eleair/MindVisionlinuxSDK' --frames 300 --timeout-ms 1500 --resolution-index 3 --exposure-us 30000

python3 scripts/test_mindvision_camera.py --config configs/camera_mv_suf1205gm.yaml --sdk-root '/home/sunrise/east_eleair/MindVisionlinuxSDK' --frames 200 --timeout-ms 1500 --resolution-index 2 --exposure-us 30000

`
### 测试结果
实际帧率约 19.2 fps（设备时间戳和读取时间都约 52 ms/帧
吞吐约 2.304 MB × 19.2 ≈ 44 MB/s，仍接近 USB 2.0 的实际带宽上限

模式         单帧大小    实测帧率    USB 2.0 结论
  ━━━━━━━━━━━  ━━━━━━━━━━  ━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━
   4096×3072    12.58 MB    3.56 fps    全分辨率可用，但延迟高
  ───────────  ──────────  ──────────  ────────────────────────
   2592×2048     5.31 MB    8.43 fps    中等速度
  ───────────  ──────────  ──────────  ────────────────────────
   1920×1200     2.30 MB    19.2 fps    推荐临时工作模式

## 对单张 bmp 图像跑完整视觉处理流程，验证读取、算法、结果 JSON 和输出写盘

`
python3 -m tan_vision.cli --config configs/default.yaml --source folder --input '/home/sunrise/east_eleair/MindVisionlinuxSDK/demo/python_demo' --output output_mindvision_bmp --max-frames 1
`

```bash
python3 scripts/test_mindvision_camera.py \
  --config configs/camera_mv_suf1205gm.yaml \
  --sdk-root /home/sunrise/east_eleair/MindVisionlinuxSDK \
  --frames 100 \
  --exposure-us 30000


```bash

ros2 bag info $HOME/east_eleair/physic_bev/bags/physic_bev/

ros2 launch inspection_bringup hardware_pipeline.launch.py start_mindvision:=true mindvision_publish_rate_hz:=10.0 start_mid360:=true start_eddy:=false start_sync:=false record:=true publish_rough_extrinsics:=false debug_mode:=true output_directory:=$HOME/east_eleair/physic_bev/bags/physic_bev

