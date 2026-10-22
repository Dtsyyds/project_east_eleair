"""Build only this checkout into .runtime/<platform>; never modifies system SDKs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--platform", choices=["s100p", "x86", "rk3588"], default=os.getenv("INSPECTION_PLATFORM", "s100p"))
    parser.add_argument("--plan", action="store_true", help="Print exact build commands without executing or creating outputs")
    parser.add_argument("--skip-native", action="store_true", help="Use explicitly provisioned system/native dependencies instead of rebuilding vendored SDKs")
    parser.add_argument("--offline", action="store_true", help="Verify system archives and unpack compatible debs into this runtime sysroot")
    parser.add_argument("--packages-select", nargs="+", help="Override profile selection with explicitly selected package names")
    args = parser.parse_args()
    profile = json.loads((ROOT / "config/platforms" / (args.platform + ".json")).read_text())
    runtime = ROOT / ".runtime" / args.platform
    env = dict(os.environ, INSPECTION_PLATFORM=args.platform, TANK_RUNTIME_ROOT=str(runtime),
               TANK_INSTALL_ROOT=str(runtime / "install"), TANK_OFFLINE_PREFIX=str(runtime / "sysroot"))
    native = runtime / "native/install"
    env["CMAKE_PREFIX_PATH"] = str(native) + ":" + str(runtime / "sysroot/usr") + ":" + str(runtime / "sysroot/opt/ros/humble") + ":" + env.get("CMAKE_PREFIX_PATH", "")
    commands = []
    if args.offline:
        commands.append([sys.executable, str(ROOT / "deploy/offline.py"), "--platform", args.platform])
    python = runtime / "venv/bin/python"
    if not python.exists():
        commands.append([sys.executable, "-m", "venv", "--system-site-packages", str(runtime / "venv")])
    # Reuse the recorded D405/system versions; never fetch or change system packages.
    commands.append([str(python), "-m", "pip", "install", "--no-index", "--no-deps",
                     "--find-links", str(ROOT / "third_party/offline/d405"),
                     "--find-links", str(ROOT / "third_party/offline/system/wheels"),
                     "numpy==1.26.4", "opencv-python==4.11.0.86", "PyYAML==6.0.3", "websockets==15.0.1"])
    for package in ["packages/emat_core", "packages/inspection_runtime", "packages/tank_vision", "packages/d405_seam_tracking", "apps/inspection_cli", "apps/web_console"]:
        commands.append([str(python), "-m", "pip", "install", "--no-index", "--no-deps", "--no-build-isolation", "-e", str(ROOT / package)])
    if not args.skip_native:
        for name, flags in [("Livox-SDK2", ["-DLIVOX_BUILD_SAMPLES=OFF"]), ("Sophus", ["-DBUILD_SOPHUS_TESTS=OFF", "-DBUILD_SOPHUS_EXAMPLES=OFF"]), ("librealsense", ["-DBUILD_EXAMPLES=OFF", "-DBUILD_GRAPHICAL_EXAMPLES=OFF", "-DBUILD_UNIT_TESTS=OFF", "-DBUILD_TOOLS=OFF", "-DBUILD_GLSL_EXTENSIONS=OFF"])]:
            build = runtime / "native/build" / name
            commands += [["cmake", "-S", str(ROOT / "third_party" / name), "-B", str(build), "-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_INSTALL_PREFIX=" + str(native)] + flags,
                         ["cmake", "--build", str(build), "--parallel", str(os.cpu_count() or 2)],
                         ["cmake", "--install", str(build)]]
    selected = args.packages_select or profile["packages"]
    import xml.etree.ElementTree as ET
    available = {ET.parse(p).getroot().findtext("name") for p in (ROOT / "ros2_ws/src").rglob("package.xml")}
    if set(selected) - available: parser.error("Unknown packages: " + str(sorted(set(selected) - available)))
    command = ["colcon", "--log-base", str(runtime / "log"), "build", "--base-paths", str(ROOT / "ros2_ws/src"),
               "--build-base", str(runtime / "build"), "--install-base", str(runtime / "install"),
               "--packages-up-to", *selected, "--cmake-args", "-DBUILD_TESTING=ON", "-DFAST_LIVO_OUTPUT_ROOT=" + str(runtime / "fast_livo"), "-DPython3_EXECUTABLE=" + str(python), "-DPYTHON_EXECUTABLE=" + str(python)]
    if not args.skip_native:
        command += ["-DLIVOX_LIDAR_SDK_LIBRARY=" + str(native / "lib/liblivox_lidar_sdk_shared.so"),
                    "-DLIVOX_LIDAR_SDK_INCLUDE_DIR=" + str(ROOT / "third_party/Livox-SDK2/include")]
    commands.append(command)
    if args.plan:
        print(json.dumps({"platform": args.platform, "runtime": str(runtime), "commands": commands, "excluded": profile["excluded_packages"] if not args.packages_select else {}}, indent=2))
        return
    # Re-source selected platform, including newly unpacked offline resources, for colcon.
    for command in commands:
        if command[0] == "colcon":
            subprocess.run(["bash", "-c", 'source "$1/deploy/env.sh" "$2"; shift 2; exec "$@"', "inspection-build", str(ROOT), args.platform, *command], cwd=ROOT, env=env, check=True)
        else:
            command_env = dict(env)
            # setuptools develop must write its .pth even when env.sh exposes source.
            # Otherwise installation appears successful but breaks in a clean shell.
            if len(command) > 2 and command[1:3] == ["-m", "pip"]:
                command_env.pop("PYTHONPATH", None)
            subprocess.run(command, cwd=ROOT, env=command_env, check=True)

if __name__ == "__main__":
    main()
