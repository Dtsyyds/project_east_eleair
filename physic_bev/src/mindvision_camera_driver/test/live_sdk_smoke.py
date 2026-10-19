#!/usr/bin/env python3
"""Bounded real-camera smoke test using the official MindVision Python API."""

import argparse
import ctypes
import json
import time

from mindvision_camera_driver.node import load_mvsdk


def checked(sdk, operation, *args):
    status = operation(*args)
    if status != sdk.CAMERA_STATUS_SUCCESS:
        raise RuntimeError(f"{operation.__name__} failed with status {status}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-root", required=True)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--frames", type=int, default=30)
    args = parser.parse_args()
    sdk, root, library = load_mvsdk(args.sdk_root)
    checked(sdk, sdk.CameraSdkInit, 1)
    devices = list(sdk.CameraEnumerateDevice())
    matches = [device for device in devices if device.GetSn() == args.serial]
    if len(matches) != 1:
        raise RuntimeError(f"serial {args.serial!r} matched {len(matches)} devices")
    device = matches[0]
    handle = None
    output = 0
    playing = False
    received = 0
    released = 0
    first_head = None
    sample = None
    started = time.monotonic()
    try:
        handle = sdk.CameraInit(device, -1, -1)
        capability = sdk.CameraGetCapability(handle)
        checked(sdk, sdk.CameraSetIspOutFormat, handle, sdk.CAMERA_MEDIA_TYPE_BGR8)
        checked(sdk, sdk.CameraSetTriggerMode, handle, 0)
        checked(sdk, sdk.CameraPlay, handle)
        playing = True
        size = capability.sResolutionRange.iWidthMax * capability.sResolutionRange.iHeightMax * 3
        output = sdk.CameraAlignMalloc(size, 16)
        if not output:
            raise RuntimeError("CameraAlignMalloc returned null")
        while received < args.frames:
            raw = None
            try:
                raw, head = sdk.CameraGetImageBuffer(handle, 2000)
                checked(sdk, sdk.CameraImageProcess, handle, raw, output, head)
                if first_head is None:
                    first_head = head
                    view = (ctypes.c_ubyte * int(head.uBytes)).from_address(output)
                    stride = max(1, int(head.uBytes) // 4096)
                    values = list(view[::stride])
                    sample = {
                        "count": len(values), "min": min(values), "max": max(values),
                        "mean": round(sum(values) / len(values), 3),
                    }
                received += 1
            finally:
                if raw is not None:
                    checked(sdk, sdk.CameraReleaseImageBuffer, handle, raw)
                    released += 1
    finally:
        if handle is not None and playing:
            checked(sdk, sdk.CameraPause, handle)
        if output:
            sdk.CameraAlignFree(output)
        if handle is not None:
            checked(sdk, sdk.CameraUnInit, handle)
    elapsed = time.monotonic() - started
    result = {
        "sdk_version": sdk.CameraSdkGetVersionString(), "sdk_root": str(root),
        "sdk_library": str(library), "model": device.GetFriendlyName(),
        "serial": device.GetSn(), "port": device.GetPortType(),
        "frames_received": received, "buffers_released": released,
        "elapsed_sec": round(elapsed, 3), "fps": round(received / elapsed, 3),
        "width": int(first_head.iWidth), "height": int(first_head.iHeight),
        "media_type": int(first_head.uiMediaType), "bytes": int(first_head.uBytes),
        "sample": sample,
    }
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return int(received != args.frames or released != args.frames or sample["min"] == sample["max"])


if __name__ == "__main__":
    raise SystemExit(main())
