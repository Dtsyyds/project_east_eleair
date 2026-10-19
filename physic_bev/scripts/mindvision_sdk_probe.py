#!/usr/bin/env python3
"""Minimal, non-ROS MindVision SDK probe with machine-readable diagnostics.

The probe stops after the first failed SDK boundary. It never changes firmware,
installs udev rules, or substitutes a different SDK binary.
"""

from __future__ import annotations

import argparse
import ctypes
import importlib
import json
import os
from pathlib import Path
import platform
import stat
import subprocess
import sys
import time
from typing import Any


def command(*argv: str) -> dict[str, Any]:
    try:
        completed = subprocess.run(argv, capture_output=True, text=True, timeout=10, check=False)
        return {"argv": list(argv), "returncode": completed.returncode,
                "stdout": completed.stdout.strip(), "stderr": completed.stderr.strip()}
    except (OSError, subprocess.SubprocessError) as exc:
        return {"argv": list(argv), "returncode": 127, "error": str(exc)}


def step(report: dict[str, Any], name: str, status: str, **details: Any) -> None:
    record = {"name": name, "status": status, **details}
    report["steps"].append(record)
    print(json.dumps(record, ensure_ascii=False), flush=True)


def sdk_arch_dir(root: Path) -> Path:
    machine = platform.machine().lower()
    mapping = {"aarch64": "arm64", "arm64": "arm64", "x86_64": "x64", "amd64": "x64"}
    if machine not in mapping:
        raise RuntimeError(f"unsupported host architecture: {machine}")
    return root / "lib" / mapping[machine]


def load_sdk(root: Path):
    library = sdk_arch_dir(root) / "libMVSDK.so"
    binding_dir = root / "demo" / "python_demo"
    binding = binding_dir / "mvsdk.py"
    if not library.is_file() or not binding.is_file():
        raise FileNotFoundError(f"SDK requires {library} and {binding}")
    ctypes.CDLL(str(library), mode=ctypes.RTLD_GLOBAL)
    sys.path.insert(0, str(binding_dir))
    return importlib.import_module("mvsdk"), library


def device_json(device: Any) -> dict[str, str]:
    return {"friendly_name": str(device.GetFriendlyName()),
            "serial_number": str(device.GetSn()), "port_type": str(device.GetPortType())}


def capability_json(capability: Any) -> dict[str, Any]:
    resolutions = []
    for index in range(int(capability.iImageSizeDesc)):
        item = capability.pImageSizeDesc[index]
        resolutions.append({"index": index, "sdk_index": int(item.iIndex),
                            "width": int(item.iWidth), "height": int(item.iHeight),
                            "description": str(item.GetDescription())})
    media_types = []
    for index in range(int(capability.iMediaTypeDesc)):
        item = capability.pMediaTypeDesc[index]
        media_types.append({"index": index, "media_type": int(item.iMediaType),
                            "description": str(item.GetDescription())})
    frame_speeds = []
    for index in range(int(capability.iFrameSpeedDesc)):
        item = capability.pFrameSpeedDesc[index]
        frame_speeds.append({"index": index, "sdk_index": int(item.iIndex),
                             "description": str(item.GetDescription())})
    expose = capability.sExposeDesc
    return {"mono_sensor": bool(capability.sIspCapacity.bMonoSensor),
            "max_width": int(capability.sResolutionRange.iWidthMax),
            "max_height": int(capability.sResolutionRange.iHeightMax),
            "resolutions": resolutions, "media_types": media_types,
            "frame_speeds": frame_speeds,
            "analog_gain_raw_range": [int(expose.uiAnalogGainMin), int(expose.uiAnalogGainMax)],
            "analog_gain_step": float(expose.fAnalogGainStep)}


def select_device(devices: list[Any], serial: str) -> Any:
    if serial:
        matches = [device for device in devices if str(device.GetSn()) == serial]
        if len(matches) != 1:
            raise RuntimeError(f"serial {serial!r} matched {len(matches)} enumerated devices")
        return matches[0]
    if len(devices) != 1:
        raise RuntimeError(f"serial is required when SDK enumerates {len(devices)} devices")
    return devices[0]


def main(argv: list[str] | None = None) -> int:
    workspace = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", type=Path, default=Path(os.environ.get(
        "MINDVISION_SDK_ROOT", workspace / "MindVisionlinuxSDK")))
    parser.add_argument("--serial", default="046041920527")
    parser.add_argument("--output-image", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--timeout-ms", type=int, default=3000)
    args = parser.parse_args(argv)
    sdk_root = args.sdk_root.expanduser().resolve()
    output_image = (args.output_image or Path("/tmp/mindvision_sdk_probe.bmp")).resolve()
    report: dict[str, Any] = {
        "schema_version": "mindvision_sdk_probe_v1", "started_unix_ns": time.time_ns(),
        "host_architecture": platform.machine(), "sdk_root": str(sdk_root),
        "requested_serial": args.serial, "output_image": str(output_image),
        "steps": [], "passed": False}
    handle = 0
    frame_buffer = 0
    raw_buffer = None
    sdk = None
    return_code = 1
    try:
        sdk, library = load_sdk(sdk_root)
        report["library"] = {"path": str(library), "mode": stat.filemode(library.stat().st_mode),
                             "file": command("file", str(library)),
                             "dependencies": command("ldd", str(library))}
        step(report, "load_sdk", "PASS", library=str(library),
             version=str(sdk.CameraSdkGetVersionString()))
        init_code = int(sdk.CameraSdkInit(1))
        if init_code != int(sdk.CAMERA_STATUS_SUCCESS):
            step(report, "sdk_init", "FAIL", error_code=init_code)
            return_code = 2
            return return_code
        step(report, "sdk_init", "PASS", error_code=init_code)
        devices = list(sdk.CameraEnumerateDevice())
        enumerate_code = int(sdk.GetLastError()) if hasattr(sdk, "GetLastError") else None
        report["enumerated_devices"] = [device_json(device) for device in devices]
        if not devices:
            step(report, "enumerate", "BLOCKED", error_code=enumerate_code, device_count=0)
            return_code = 3
            return return_code
        step(report, "enumerate", "PASS", error_code=enumerate_code,
             devices=report["enumerated_devices"])
        device = select_device(devices, args.serial)
        try:
            handle = sdk.CameraInit(device, -1, -1)
        except sdk.CameraException as exc:
            step(report, "open", "FAIL", error_code=int(exc.error_code),
                 error_message=str(exc.message))
            return_code = 4
            return return_code
        step(report, "open", "PASS", handle=int(handle), device=device_json(device))
        capability = sdk.CameraGetCapability(handle)
        capabilities = capability_json(capability)
        report["capabilities"] = capabilities
        step(report, "query_capability", "PASS", capabilities=capabilities)
        mono = capabilities["mono_sensor"]
        output_format = sdk.CAMERA_MEDIA_TYPE_MONO8 if mono else sdk.CAMERA_MEDIA_TYPE_BGR8
        output_encoding = "mono8" if mono else "bgr8"
        status = int(sdk.CameraSetIspOutFormat(handle, output_format))
        if status != int(sdk.CAMERA_STATUS_SUCCESS):
            step(report, "configure_output", "FAIL", error_code=status, encoding=output_encoding)
            return_code = 5
            return return_code
        trigger_status = int(sdk.CameraSetTriggerMode(handle, 0))
        play_status = int(sdk.CameraPlay(handle))
        if trigger_status != 0 or play_status != 0:
            step(report, "start_acquisition", "FAIL", trigger_error_code=trigger_status,
                 play_error_code=play_status)
            return_code = 6
            return return_code
        step(report, "start_acquisition", "PASS", encoding=output_encoding)
        channels = 1 if mono else 3
        frame_buffer = sdk.CameraAlignMalloc(
            capabilities["max_width"] * capabilities["max_height"] * channels, 16)
        if not frame_buffer:
            step(report, "allocate_frame", "FAIL", error="CameraAlignMalloc returned null")
            return_code = 7
            return return_code
        try:
            raw_buffer, frame = sdk.CameraGetImageBuffer(handle, args.timeout_ms)
            process_status = int(sdk.CameraImageProcess(handle, raw_buffer, frame_buffer, frame))
        except sdk.CameraException as exc:
            step(report, "capture_frame", "FAIL", error_code=int(exc.error_code),
                 error_message=str(exc.message))
            return_code = 8
            return return_code
        finally:
            if raw_buffer is not None:
                sdk.CameraReleaseImageBuffer(handle, raw_buffer)
                raw_buffer = None
        if process_status != 0:
            step(report, "capture_frame", "FAIL", error_code=process_status,
                 stage="CameraImageProcess")
            return_code = 9
            return return_code
        step(report, "capture_frame", "PASS", width=int(frame.iWidth),
             height=int(frame.iHeight), bytes=int(frame.uBytes))
        output_image.parent.mkdir(parents=True, exist_ok=True)
        save_status = int(sdk.CameraSaveImage(
            handle, str(output_image), frame_buffer, frame, sdk.FILE_BMP, 100))
        if save_status != 0 or not output_image.is_file():
            step(report, "save_frame", "FAIL", error_code=save_status, image=str(output_image))
            return_code = 10
            return return_code
        step(report, "save_frame", "PASS", error_code=save_status, image=str(output_image),
             size=output_image.stat().st_size)
        report["passed"] = True
        return_code = 0
        return return_code
    except Exception as exc:  # vendor boundary: keep the exact failure
        step(report, "probe", "FAIL", error_type=type(exc).__name__, error=str(exc))
        return_code = 1
        return return_code
    finally:
        if sdk is not None and raw_buffer is not None and handle:
            try:
                sdk.CameraReleaseImageBuffer(handle, raw_buffer)
            except Exception:
                pass
        if sdk is not None and frame_buffer:
            try:
                sdk.CameraAlignFree(frame_buffer)
            except Exception:
                pass
        if sdk is not None and handle:
            try:
                close_status = int(sdk.CameraUnInit(handle))
                step(report, "close", "PASS" if close_status == 0 else "FAIL",
                     error_code=close_status)
            except Exception as exc:
                step(report, "close", "FAIL", error=str(exc))
        report["finished_unix_ns"] = time.time_ns()
        report["result_code"] = return_code
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
