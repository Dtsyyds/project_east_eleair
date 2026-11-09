"""Read-only dependency preflight: never opens devices or starts ROS nodes."""
import argparse
import importlib.util
import importlib.metadata
import json
import os
import platform
import shutil
from .paths import project_root, config_path, runtime_root

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="Emit machine-readable status (also the default)")
    args = parser.parse_args()
    root = project_root()
    checks = {}
    for name in ["tank_vision", "d405_seam_tracking", "emat_core", "inspection_runtime", "rclpy", "rosbag2_py", "inspection_operations", "yaml", "cv2", "numpy", "serial"]:
        try:
            spec = importlib.util.find_spec(name)
            checks[name] = {"status": "available" if spec else "dependency_blocked", "origin": spec.origin if spec else None}
        except (ImportError, ValueError) as exc:
            checks[name] = {"status": "dependency_blocked", "error": str(exc)}
    for name, expected in {"numpy": "1.26.4", "opencv-python": "4.11.0.86", "PyYAML": "6.0.3"}.items():
        try:
            found = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            found = None
        checks["version:" + name] = {"status": "available" if found == expected else "dependency_blocked", "expected": expected, "found": found}
    rerun_python = __import__("pathlib").Path(os.environ.get("FIELD_RRD_PYTHON", runtime_root() / "rerun_venv/bin/python"))
    checks["rerun_environment"] = {"status": "available" if rerun_python.is_file() else "dependency_blocked", "path": str(rerun_python), "note": "existence only, SDK version/ABI still requires validation"}
    for name in ["colcon", "cmake", "ffmpeg", "ros2"]:
        checks[name] = {"status": "available" if shutil.which(name) else "dependency_blocked", "path": shutil.which(name)}
    sdk = __import__("pathlib").Path(os.environ.get("MINDVISION_SDK_ROOT", root / "third_party/mindvision/2.1.0.50"))
    arch = "arm64" if platform.machine() in ("aarch64", "arm64") else "x64"
    for name, path in {"mindvision_library": sdk / "lib" / arch / "libMVSDK.so", "mindvision_binding": sdk / "demo/python_demo/mvsdk.py", "system_config": config_path("communication/system.yaml"), "ros_overlay": runtime_root() / "install/local_setup.bash"}.items():
        checks[name] = {"status": "available" if path.is_file() else "dependency_blocked", "path": str(path)}
    print(json.dumps({"checks": checks, "hardware": "not_verified", "build": "not_executed_by_preflight", "model_and_calibration": "see config/README.md"}, indent=2))
    return 0 if all(v["status"] == "available" for v in checks.values()) else 2
if __name__ == "__main__": raise SystemExit(main())
