QT_QPA_PLATFORM=xcb /usr/bin/python3 - <<'PY'
import sys
import cv2
import numpy as np

SDK = "/home/sunrise/east_eleair/MindVisionlinuxSDK/demo/python_demo"
sys.path.insert(0, SDK)
import mvsdk

handle = 0
frame_buffer = 0
raw_buffer = None

try:
    devices = list(mvsdk.CameraEnumerateDevice())
    if not devices:
        raise RuntimeError("未发现 MindVision 相机")

    device = devices[0]
    print("Camera:", device.GetFriendlyName(), device.GetPortType())

    handle = mvsdk.CameraInit(device, -1, -1)
    capability = mvsdk.CameraGetCapability(handle)
    is_mono = bool(capability.sIspCapacity.bMonoSensor)

    output_format = (
        mvsdk.CAMERA_MEDIA_TYPE_MONO8
        if is_mono
        else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    )
    mvsdk.CameraSetIspOutFormat(handle, output_format)
    mvsdk.CameraSetTriggerMode(handle, 0)

    # 调整镜头期间使用自动曝光
    mvsdk.CameraSetAeState(handle, 1)
    mvsdk.CameraPlay(handle)

    channels = 1 if is_mono else 3
    buffer_size = (
        capability.sResolutionRange.iWidthMax
        * capability.sResolutionRange.iHeightMax
        * channels
    )
    frame_buffer = mvsdk.CameraAlignMalloc(buffer_size, 16)

    window = "MindVision live - q/ESC to exit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 960, 720)

    while True:
        try:
            raw_buffer, head = mvsdk.CameraGetImageBuffer(handle, 1000)
            mvsdk.CameraImageProcess(handle, raw_buffer, frame_buffer, head)
            mvsdk.CameraReleaseImageBuffer(handle, raw_buffer)
            raw_buffer = None

            size = int(head.iWidth) * int(head.iHeight) * channels
            pixels = (mvsdk.c_ubyte * size).from_address(frame_buffer)
            frame = np.frombuffer(pixels, dtype=np.uint8)

            if is_mono:
                frame = frame.reshape(head.iHeight, head.iWidth)
            else:
                frame = frame.reshape(head.iHeight, head.iWidth, 3)

            preview = cv2.resize(frame, (960, 720), interpolation=cv2.INTER_AREA)
            cv2.imshow(window, preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break

        except mvsdk.CameraException as exc:
            if exc.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                raise

finally:
    if raw_buffer is not None and handle:
        mvsdk.CameraReleaseImageBuffer(handle, raw_buffer)
    if handle:
        mvsdk.CameraUnInit(handle)
    if frame_buffer:
        mvsdk.CameraAlignFree(frame_buffer)
    cv2.destroyAllWindows()
PY
