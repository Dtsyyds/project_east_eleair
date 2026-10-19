"""Shared validation and manifest helpers for the visual field run launch."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import socket
import subprocess
from typing import Any


RUN_SUBDIRECTORIES = (
    "mcap", "live_session", "failure_snapshots", "logs", "frozen_configs",
)


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _command(*args: str) -> str | None:
    try:
        result = subprocess.run(args, check=False, capture_output=True, text=True, timeout=3)
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() if result.returncode == 0 else None


def filesystem_info(path: str | Path) -> dict[str, Any]:
    target = Path(path).resolve()
    device = filesystem = mountpoint = None
    findmnt = _command("findmnt", "-n", "-o", "SOURCE,FSTYPE,TARGET", "--target", str(target))
    if findmnt:
        fields = findmnt.split(None, 2)
        if len(fields) == 3:
            device, filesystem, mountpoint = fields
    usage = shutil.disk_usage(target)
    return {
        "path": str(target), "device": device, "filesystem": filesystem,
        "mountpoint": mountpoint, "is_mount": os.path.ismount(target),
        "total_bytes": usage.total, "free_bytes": usage.free,
    }


def validate_data_root(data_root: str | Path, min_free_bytes: int = 1_000_000_000) -> dict[str, Any]:
    root = Path(data_root).expanduser().resolve()
    if not root.is_dir():
        raise RuntimeError(f"data_root does not exist or is not a directory: {root}")
    info = filesystem_info(root)
    if not info["is_mount"]:
        raise RuntimeError(f"data_root is not a mount point; refusing system-disk capture: {root}")
    if int(info["free_bytes"]) < int(min_free_bytes):
        raise RuntimeError(
            f"insufficient free space at {root}: {info['free_bytes']} < {min_free_bytes} bytes")
    if not os.access(root, os.W_OK):
        raise RuntimeError(f"data_root is not writable: {root}")
    return info


def prepare_run(data_root: str | Path, run_id: str) -> tuple[Path, dict[str, Any]]:
    root = Path(data_root).expanduser().resolve()
    filesystem = validate_data_root(root)
    run_name = run_id.strip() or datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    if any(char not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for char in run_name):
        raise RuntimeError("run_id may contain only ASCII letters, digits, '_' and '-'")
    run_dir = root / "runs" / run_name
    if run_dir.exists() and any(run_dir.iterdir()):
        raise RuntimeError(f"run_id directory already exists and is non-empty: {run_dir}")
    run_dir.mkdir(parents=True, exist_ok=True)
    for name in RUN_SUBDIRECTORIES:
        (run_dir / name).mkdir(exist_ok=True)
    return run_dir, filesystem


def system_evidence() -> dict[str, Any]:
    return {
        "hostname": socket.gethostname(), "kernel": platform.release(),
        "architecture": platform.machine(), "platform": platform.platform(),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        "ros_localhost_only": os.environ.get("ROS_LOCALHOST_ONLY", "0"),
        "git": {
            "tank_vision_inspection": _git_evidence(Path(__file__).resolve().parents[4] / "tank_vision_inspection"),
            "physic_bev_src": _git_evidence(Path(__file__).resolve().parents[1].parents[1]),
        },
    }


def _git_evidence(path: Path) -> dict[str, Any]:
    return {
        "path": str(path), "head": _command("git", "-C", str(path), "rev-parse", "HEAD"),
        "dirty": bool(_command("git", "-C", str(path), "status", "--porcelain")),
        "dirty_summary": (_command("git", "-C", str(path), "status", "--short") or "")[:4000],
    }


def write_manifest(run_dir: str | Path, *, run_id: str, data_root: str | Path,
                   tracker_config: str | Path, tracker_reference: str | Path,
                   camera: dict[str, Any], exposure: dict[str, Any],
                   topics: list[str], launch_parameters: dict[str, Any],
                   camera_info_valid: bool | None = None) -> Path:
    run_dir = Path(run_dir).resolve()
    manifest = {
        "schema_version": "field_visual_v1", "run_id": run_id,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "software_evidence_level": "field_validation",
        **system_evidence(), "data_root": str(Path(data_root).resolve()),
        "ssd": filesystem_info(data_root), "camera": camera,
        "exposure": exposure, "topics": topics,
        "tracker": {"config": str(Path(tracker_config).resolve()),
                     "config_sha256": sha256_file(tracker_config),
                     "reference": str(Path(tracker_reference).resolve()),
                     "reference_sha256": sha256_file(tracker_reference)},
        "roi": None, "camera_info_valid": camera_info_valid,
        "rough_extrinsics_enabled": False, "launch_parameters": launch_parameters,
        "status": {"complete": False, "finalized": False, "validation": "pending",
                    "valid_for_formal_validation": False},
    }
    path = run_dir / "run_manifest.json"
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (run_dir / "field_notes.md").write_text(
        f"# {run_id}\n\nEvidence level: `field_validation`\n\n"
        "CameraInfo is expected to be uncalibrated for this pixel-only phase.\n",
        encoding="utf-8")
    return path

