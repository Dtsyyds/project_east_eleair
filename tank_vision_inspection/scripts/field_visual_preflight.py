#!/usr/bin/env python3
"""Read-only preflight and source/install evidence for a visual field run."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import socket
import subprocess
import sys
import tempfile
import time


SERIAL = "046081820650"
ROOT = Path(__file__).resolve().parents[1]
BEV = ROOT.parent / "physic_bev"
TRACKER = ROOT / "ros2_ws/src/tank_vision_ros/tank_vision_ros/stable_seam_node.py"
CORE_TRACKER = ROOT / "tank_vision/stable_seam_tracker.py"
CONFIG = ROOT / "configs/s100p_guided_anchor_seam_4096x3072.yaml"
REFERENCE = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"
LAUNCH = BEV / "src/inspection_bringup/launch/visual_field_validation.launch.py"
MSG = ROOT / "ros2_ws/src/tank_vision_interfaces/msg/SeamGeometry.msg"


def sha(path: Path) -> str | None:
    if not path.is_file():
        return None
    h = hashlib.sha256()
    with path.open("rb") as f:
        for b in iter(lambda: f.read(4 * 1024 * 1024), b""):
            h.update(b)
    return h.hexdigest()


def run(*args: str, timeout: float = 5.0) -> tuple[int, str]:
    try:
        p = subprocess.run(args, capture_output=True, text=True, timeout=timeout, check=False)
        return p.returncode, (p.stdout + p.stderr).strip()
    except (OSError, subprocess.SubprocessError) as e:
        return 127, str(e)


def resolved_import(module: str) -> str | None:
    code = "import importlib, pathlib; m=importlib.import_module(%r); print(pathlib.Path(m.__file__).resolve())" % module
    rc, out = run(sys.executable, "-c", code)
    return out.splitlines()[-1] if rc == 0 and out else None


def mount_info(path: Path) -> dict:
    rc, out = run("findmnt", "-T", str(path), "-o", "SOURCE,TARGET,FSTYPE,OPTIONS")
    rc2, disk = run("df", "-hT", str(path))
    rc3, lsblk = run("lsblk", "-o", "NAME,MODEL,SERIAL,SIZE,FSTYPE,LABEL,MOUNTPOINTS,RO")
    return {"findmnt": out, "df": disk, "lsblk": lsblk, "command_ok": rc == rc2 == rc3 == 0}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", default="/media/mcap_data")
    ap.add_argument("--output", required=True)
    ap.add_argument("--manifest-output", default="")
    ap.add_argument("--run-id", default="")
    args = ap.parse_args()
    data_root = Path(args.data_root).expanduser().resolve()
    checks: list[dict] = []

    def check(name: str, passed: bool, detail: object, severity: str = "error") -> None:
        checks.append({"name": name, "passed": bool(passed), "severity": severity, "detail": detail})

    check("data_root_exists", data_root.is_dir(), str(data_root))
    mi = mount_info(data_root) if data_root.is_dir() else {}
    check("mount_audit", bool(mi.get("command_ok")), mi, "error")
    try:
        usage = shutil.disk_usage(data_root)
        check("free_space", usage.free >= 1_000_000_000, {"free_bytes": usage.free, "total_bytes": usage.total})
    except OSError as e:
        check("free_space", False, str(e))
    probe = data_root / f".field_visual_preflight_{os.getpid()}_{time.time_ns()}.tmp"
    writable = False
    try:
        with probe.open("wb") as f:
            f.write(b"field_visual_preflight\n")
            f.flush(); os.fsync(f.fileno())
        writable = True
    except OSError as e:
        check("data_root_write", False, str(e))
    finally:
        try: probe.unlink()
        except OSError: pass
    if writable: check("data_root_write", True, "fsync probe succeeded")

    check("ros_distro", os.environ.get("ROS_DISTRO") == "humble", os.environ.get("ROS_DISTRO"))
    check("mindvision_usb", SERIAL in run("lsusb")[1] or "MindVision" in run("lsusb")[1], run("lsusb")[1], "error")
    check("sdk_root", (Path(os.environ.get("MINDVISION_SDK_ROOT", "/home/sunrise/east_eleair/MindVisionlinuxSDK"))).exists(), os.environ.get("MINDVISION_SDK_ROOT"))
    for name, path in (("tracker_node_source", TRACKER), ("core_tracker_source", CORE_TRACKER), ("config_source", CONFIG), ("reference_source", REFERENCE), ("launch_source", LAUNCH), ("message_source", MSG)):
        check(name, path.is_file(), {"path": str(path), "sha256": sha(path)})
    source_node = str(TRACKER.resolve())
    resolved_node = resolved_import("tank_vision_ros.stable_seam_node")
    source_core = str(CORE_TRACKER.resolve())
    resolved_core = resolved_import("tank_vision.stable_seam_tracker")
    check("tracker_node_source_install_match", resolved_node == source_node, {"source": source_node, "resolved": resolved_node})
    check("core_tracker_source_install_match", resolved_core == source_core, {"source": source_core, "resolved": resolved_core})
    rc, prefix = run("ros2", "pkg", "prefix", "inspection_bringup")
    check("inspection_bringup_discoverable", rc == 0, prefix)
    installed_launch = Path(prefix) / "share/inspection_bringup/launch/visual_field_validation.launch.py" if rc == 0 else None
    check("launch_source_install_match", bool(installed_launch and installed_launch.is_file() and sha(installed_launch) == sha(LAUNCH)), {"source": str(LAUNCH), "source_sha256": sha(LAUNCH), "install": str(installed_launch) if installed_launch else None, "install_sha256": sha(installed_launch) if installed_launch else None})
    installed_topics = Path(prefix) / "share/inspection_bringup/config/record_topics_visual.yaml" if rc == 0 else None
    source_topics = BEV / "src/inspection_bringup/config/record_topics_visual.yaml"
    check("visual_record_config_source_install_match", bool(installed_topics and installed_topics.is_file() and sha(installed_topics) == sha(source_topics)), {"source": str(source_topics), "source_sha256": sha(source_topics), "install": str(installed_topics) if installed_topics else None, "install_sha256": sha(installed_topics) if installed_topics else None})
    rc, nodes = run("ros2", "node", "list")
    check("no_existing_camera_owner", "mindvision_camera_node" not in nodes and "/camera/front" not in nodes, nodes, "error")
    rc, iface = run("ros2", "interface", "show", "tank_vision_interfaces/msg/SeamGeometry")
    check("seam_geometry_interface", rc == 0, iface[:2000])
    check("rosbag_mcap_plugin", run("ros2", "bag", "info", "--help")[0] == 0, "ros2 bag info available")
    check("ros_domain", True, {"ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"), "ROS_LOCALHOST_ONLY": os.environ.get("ROS_LOCALHOST_ONLY", "0")}, "warning")
    check("clock_cpu_memory", True, {"time": time.time(), "cpu": run("sh", "-c", "awk '{print $1}' /proc/loadavg")[1], "memory": run("free", "-h")[1]}, "warning")

    mount_options = str(mi.get("findmnt", ""))
    report = {"schema_version": "field_visual_preflight_v2", "created_at": time.time(), "run_id": args.run_id,
              "host": {"hostname": socket.gethostname(), "kernel": platform.release(), "architecture": platform.machine()},
              "data_root": str(data_root), "mount": mi,
              "execution_environment_mount_read_only": " ro," in f" {mount_options} " or mount_options.endswith(" ro"),
              "host_mount_writeability": "not inferable from this execution environment when the bind is read-only",
              "camera": {"model": "MindVision SUF1205GC", "serial": SERIAL, "width": 4096, "height": 3072, "encoding": "bgr8"},
              "source_install": {"tracker_node_source": source_node, "tracker_node_resolved": resolved_node, "core_tracker_source": source_core, "core_tracker_resolved": resolved_core, "source_install_match": all(c["passed"] for c in checks if "source_install_match" in c["name"])},
              "checks": checks, "ok": all(c["passed"] for c in checks if c["severity"] == "error")}
    out = Path(args.output).expanduser().resolve()
    try:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    except OSError as exc:
        # A read-only data_root must still produce a local diagnostic, but it
        # must never be treated as formal field evidence.
        fallback = ROOT / "output/field_w3_system_e2e_closeout/preflight_blocked.json"
        fallback.parent.mkdir(parents=True, exist_ok=True)
        report["report_write_error"] = str(exc)
        fallback.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(json.dumps({"ok": False, "output": str(fallback), "reason": str(exc)}, ensure_ascii=False))
        return 2
    txt = out.with_suffix(".txt")
    txt.write_text("FIELD VISUAL PREFLIGHT\n" + "\n".join(f"[{('PASS' if c['passed'] else 'FAIL')}] {c['name']}: {c['detail']}" for c in checks) + "\n", encoding="utf-8")
    if args.manifest_output:
        Path(args.manifest_output).expanduser().resolve().write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"ok": report["ok"], "output": str(out), "source_install_match": report["source_install"]["source_install_match"]}, ensure_ascii=False))
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
