#!/usr/bin/env python3
"""Preflight checks for the single-camera visual field-validation run."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys
import time

COMMON = Path(__file__).resolve().parents[1] / "src" / "inspection_bringup" / "launch"
sys.path.insert(0, str(COMMON))
from field_validation_common import filesystem_info, sha256_file  # noqa: E402


def command(*args: str, timeout: float = 8.0) -> tuple[int, str, str]:
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=timeout, check=False)
    except (OSError, subprocess.SubprocessError) as exc:
        return 127, "", str(exc)
    return result.returncode, result.stdout.strip(), result.stderr.strip()


def check_ros_package(package: str) -> dict[str, object]:
    code, out, err = command("ros2", "pkg", "prefix", package)
    return {"package": package, "available": code == 0, "prefix": out or None, "error": err or None}


def check_sdk(root: Path, expected_serial: str) -> dict[str, object]:
    result: dict[str, object] = {
        "root": str(root), "architecture": platform.machine(), "available": False,
        "software_status": "missing", "hardware_status": "not_checked",
        "sdk_loaded": False, "sdk_initialized": False,
        "camera_enumeration": "pending", "cameras": [],
    }
    binding = root / "demo" / "python_demo" / "mvsdk.py"
    library_name = "arm64" if platform.machine() in {"aarch64", "arm64"} else "x64"
    library = root / "lib" / library_name / "libMVSDK.so"
    result.update({"binding": str(binding), "library": str(library),
                   "binding_exists": binding.is_file(), "library_exists": library.is_file()})
    if not binding.is_file() or not library.is_file():
        return result
    try:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "mindvision_camera_driver"))
        from mindvision_camera_driver.node import load_mvsdk
        sdk, _ = load_mvsdk(str(root))
        result["sdk_loaded"] = True
        result["sdk_version"] = str(sdk.CameraSdkGetVersionString())
        init_code = int(sdk.CameraSdkInit(1))
        result["sdk_init_code"] = init_code
        if init_code != int(sdk.CAMERA_STATUS_SUCCESS):
            result["software_status"] = "init_failed"
            result["error"] = f"CameraSdkInit failed with error {init_code}"
            return result
        result.update({"available": True, "software_status": "ready", "sdk_initialized": True})
        cameras = list(sdk.CameraEnumerateDevice())
        result["enumeration_error_code"] = int(sdk.GetLastError()) if hasattr(sdk, "GetLastError") else None
        result["camera_enumeration"] = "ok" if cameras else "blocked"
        result["hardware_status"] = "ready" if cameras else "blocked"
        result["cameras"] = [{"serial_number": str(device.GetSn()), "model": str(device.GetFriendlyName())} for device in cameras]
        result["serial_match"] = (not expected_serial) or any(
            str(device.GetSn()) == expected_serial.removeprefix("sn_") for device in cameras)
    except Exception as exc:
        result["camera_enumeration"] = "error"
        result["hardware_status"] = "error" if result["sdk_initialized"] else "not_checked"
        if not result["sdk_initialized"]:
            result["software_status"] = "load_failed"
        result["error"] = f"{type(exc).__name__}: {exc}"
    return result


def topic_publishers(topic: str) -> dict[str, object]:
    code, out, err = command("ros2", "topic", "info", "-v", topic)
    match = re.search(r"Publisher count: (\d+)", out)
    return {"topic": topic, "command_ok": code == 0, "publisher_count": int(match.group(1)) if match else 0,
            "publisher_info": out or None, "error": err or None}


def camera_info_status(topic: str) -> dict[str, object]:
    code, out, err = command("ros2", "topic", "echo", topic, "--once", timeout=6.0)
    def block_values(label: str, count: int) -> list[float]:
        match = re.search(rf"(?ms)^\s*{label}:\s*(.*?)(?=^\s*[A-Za-z_]+:|\Z)", out or "")
        if not match:
            return []
        return [float(value) for value in re.findall(r"[-+]?\d+(?:\.\d+)?", match.group(1))[:count]]
    k_values = block_values("k", 9)
    p_values = block_values("p", 12)
    valid = bool(out) and len(k_values) == 9 and len(p_values) == 12 and any(k_values) and any(p_values)
    return {"topic": topic, "command_ok": code == 0, "received": bool(out), "valid": valid,
            "zero_or_unavailable_warning": not valid, "sample": out[:2000] if out else None,
            "error": err or None}


def system_status() -> dict[str, object]:
    temperatures = []
    for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            temperatures.append({"path": str(path), "celsius": int(path.read_text()) / 1000.0})
        except (OSError, ValueError):
            pass
    mem = {}
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            key, value = line.split(":", 1)
            mem[key] = value.strip()
    except OSError:
        pass
    return {"time_utc": datetime.now(timezone.utc).isoformat(), "epoch_ns": time.time_ns(),
            "temperatures": temperatures, "memory": mem, "load_average": list(os.getloadavg())}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--tracker-config", type=Path, required=True)
    parser.add_argument("--tracker-reference", type=Path, required=True)
    parser.add_argument(
        "--sdk-root",
        type=Path,
        default=Path(os.environ.get(
            "MINDVISION_SDK_ROOT",
            Path(__file__).resolve().parents[2] / "MindVisionlinuxSDK")),
    )
    parser.add_argument("--camera-serial", default="")
    parser.add_argument("--camera-topic", default="/camera/front/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/front/camera_info")
    parser.add_argument("--exposure-mode", default="auto")
    parser.add_argument("--exposure-us", type=float, default=0.0)
    parser.add_argument("--report-dir", type=Path)
    parser.add_argument("--allow-hardware-pending", action="store_true")
    args = parser.parse_args(argv)
    report_dir = (args.report_dir or (args.data_root / "runs" / args.run_id)).resolve()
    report_dir.mkdir(parents=True, exist_ok=True)
    errors: list[str] = []
    warnings: list[str] = []
    try:
        filesystem = filesystem_info(args.data_root)
        if not filesystem["is_mount"]:
            errors.append("data_root is not a mounted filesystem")
        if not os.access(args.data_root, os.W_OK):
            errors.append("data_root is not writable")
    except Exception as exc:
        filesystem = {"error": f"{type(exc).__name__}: {exc}"}
        errors.append(filesystem["error"])
    for path, label in ((args.tracker_config, "tracker_config"), (args.tracker_reference, "tracker_reference")):
        if not path.is_file():
            errors.append(f"{label} does not exist: {path}")
    hashes = {}
    if args.tracker_config.is_file():
        hashes["tracker_config_sha256"] = sha256_file(args.tracker_config)
    if args.tracker_reference.is_file():
        hashes["tracker_reference_sha256"] = sha256_file(args.tracker_reference)
    packages = [check_ros_package(name) for name in
                ("mindvision_camera_driver", "inspection_bringup", "inspection_tools", "tank_vision_ros")]
    if not os.environ.get("ROS_DISTRO"):
        errors.append("ROS_DISTRO is unset")
    if platform.machine() not in {"aarch64", "arm64", "x86_64", "amd64"}:
        errors.append(f"unsupported architecture: {platform.machine()}")
    if not all(item["available"] for item in packages):
        errors.append("one or more required ROS packages are not discoverable")
    sdk = check_sdk(args.sdk_root, args.camera_serial)
    if not sdk.get("available"):
        errors.append("MindVision SDK software is unavailable or failed to initialize")
    elif sdk.get("camera_enumeration") != "ok":
        (warnings if args.allow_hardware_pending else errors).append(
            "MindVision SDK is ready but camera enumeration is hardware/vendor blocked")
    elif not args.camera_serial and len(sdk.get("cameras", [])) != 1:
        message = "camera_serial is empty but camera enumeration did not find exactly one camera"
        (warnings if args.allow_hardware_pending else errors).append(message)
    elif args.camera_serial and not sdk.get("serial_match", False):
        message = "requested camera_serial was not found in SDK enumeration"
        (warnings if args.allow_hardware_pending else errors).append(message)
    try:
        mode = args.exposure_mode.lower()
        requested_exposure = float(args.exposure_us)
        if mode == "auto" and requested_exposure != 0:
            errors.append("auto exposure requires FIELD_EXPOSURE_US=0")
        if mode == "manual" and requested_exposure <= 0:
            errors.append("manual exposure requires FIELD_EXPOSURE_US>0")
        if mode not in {"auto", "manual"}:
            errors.append("exposure mode must be auto or manual")
    except ValueError:
        errors.append("FIELD_EXPOSURE_US is not numeric")
    camera_topics = [topic_publishers(args.camera_topic), topic_publishers(args.camera_info_topic)]
    for item in camera_topics:
        if item["publisher_count"] > 1:
            errors.append(f"multiple publishers detected for {item['topic']}")
    code, nodes, _ = command("ros2", "node", "list")
    if code == 0 and any("stable_seam_direct" in line for line in nodes.splitlines()):
        errors.append("stable_seam_direct appears to be running; stop it before field launch")
    code, direct_processes, _ = command("pgrep", "-af", "stable_seam_direct")
    if code == 0 and direct_processes:
        errors.append("stable_seam_direct process appears to own the camera")
    camera_info = camera_info_status(args.camera_info_topic)
    if camera_info.get("received") and not camera_info.get("valid"):
        warnings.append("CameraInfo K/P appears zero or uncalibrated; pixel-only tracking is allowed")
    elif not camera_info.get("received"):
        warnings.append("CameraInfo sample unavailable before camera launch")
    code, _, err = command("ros2", "bag", "record", "--storage", "mcap", "--help")
    storage_plugin = {"available": code == 0, "error": err or None}
    if code != 0:
        errors.append("MCAP storage plugin is not available")
    report = {
        "schema_version": "field_visual_preflight_v1", "created_at": datetime.now(timezone.utc).isoformat(),
        "arguments": {key: str(value) for key, value in vars(args).items()},
        "ros": {"distro": os.environ.get("ROS_DISTRO"), "domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
                "localhost_only": os.environ.get("ROS_LOCALHOST_ONLY", "0"), "packages": packages},
        "filesystem": filesystem, "hashes": hashes, "sdk": sdk, "topics": camera_topics,
        "camera_info": camera_info, "storage_plugin": storage_plugin, "system": system_status(),
        "warnings": warnings, "errors": errors, "valid": not errors,
    }
    (report_dir / "preflight_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    lines = ["FIELD VISUAL PREFLIGHT", f"valid={str(report['valid']).lower()}",
             f"data_root={args.data_root}", f"free_bytes={filesystem.get('free_bytes', 'unavailable')}" ]
    lines += [f"ERROR: {item}" for item in errors] + [f"WARNING: {item}" for item in warnings]
    (report_dir / "preflight_report.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
