"""Explicit MindVision resource paths, using the APIs declared in CameraApi.h.

No library loading or device access occurs at import time. The adapters call these
only while opening their already selected SDK/device.
"""
import ctypes
import hashlib
import os
from pathlib import Path
from .paths import config_path, runtime_root


def configure_sdk_storage(sdk):
    directory = runtime_root() / "mindvision"
    directory.mkdir(parents=True, exist_ok=True)
    setter = sdk._sdk.CameraSetDataDirectory
    setter.argtypes = [ctypes.c_char_p]
    setter.restype = ctypes.c_int
    status = setter(os.fsencode(directory))
    if status != sdk.CAMERA_STATUS_SUCCESS:
        raise RuntimeError(f"CameraSetDataDirectory failed: {status}")
    return directory


def load_serial_parameters(sdk, handle, serial):
    """Load the preserved parameter file before applying explicit node settings."""
    serial = str(serial)
    if Path(serial).name != serial or serial in ("", ".", ".."):
        raise ValueError("Invalid camera serial for parameter lookup")
    override = os.environ.get("MINDVISION_PARAMETER_FILE")
    candidates = [Path(override)] if override else [
        config_path("camera/root/Data") / (serial + ".mvdat"),
        config_path("camera/physic/Data") / (serial + ".mvdat"),
    ]
    files = [p for p in candidates if p.is_file()]
    if override and not files:
        raise FileNotFoundError(override)
    if not files:
        return None  # No prior file for this serial; retain the SDK/factory parameters.
    if len({hashlib.sha256(p.read_bytes()).digest() for p in files}) != 1:
        raise RuntimeError("Conflicting camera parameter files; select MINDVISION_PARAMETER_FILE explicitly")
    sdk.CameraReadParameterFromFile(handle, str(files[0]))
    return files[0]
