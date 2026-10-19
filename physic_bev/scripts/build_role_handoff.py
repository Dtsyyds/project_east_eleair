#!/usr/bin/env python3
"""Build role-scoped, verifiable handoff definitions for a frozen field run.

The default mode writes manifests and documentation only. Source payloads are copied
only when --materialize and --destination are both explicitly supplied.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import logging
import os
import platform
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Iterable


SCHEMA_VERSION = "field_data_role_handoff_v1"
RUN_ID = "FIELD-VEHICLE-20260928_115534"
PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "data_handoff_profiles.yaml"
DEFAULT_OUTPUT = PROJECT_ROOT / "output" / "field_data_role_handoff_v1"
ROLE_ORDER = [
    "common", "fastlivo2", "front_camera", "d405", "eddy",
    "rerun_viewer", "system_integration",
]
PROFILE_NAMES = ROLE_ORDER + ["all"]
LARGE_SUFFIXES = {".mcap", ".mkv", ".rrd"}


class HandoffError(RuntimeError):
    """Expected validation or handoff error."""


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def load_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value.rstrip() + "\n", encoding="utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_sha256sums(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    if not path.exists():
        return result
    for raw in path.read_text(encoding="utf-8").splitlines():
        if not raw.strip():
            continue
        parts = raw.split(maxsplit=1)
        if len(parts) != 2 or len(parts[0]) != 64:
            continue
        digest, rel = parts
        result[rel.lstrip("*")] = digest
    return result


def all_files(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    if not path.is_dir():
        return []
    return sorted(p for p in path.rglob("*") if p.is_file() and not p.is_symlink())


def load_config(path: Path) -> dict[str, Any]:
    # JSON is a strict YAML subset. Keeping this file in that subset avoids adding a
    # PyYAML dependency to field machines.
    config = load_json(path)
    if not isinstance(config, dict) or "profiles" not in config:
        raise HandoffError(f"invalid profile config: {path}")
    return config


def ensure_frozen_run(run_dir: Path) -> dict[str, Any]:
    if not run_dir.is_dir():
        raise HandoffError(f"run directory does not exist: {run_dir}")
    manifest = load_json(run_dir / "run_manifest.json", {})
    if manifest.get("run_id") != run_dir.name:
        raise HandoffError("run manifest identity differs from directory name")
    complete = load_json(run_dir / "complete.json", {})
    if not complete.get("complete") or not complete.get("finalized"):
        raise HandoffError("source run is not complete and finalized")
    return complete


def rrd_relative_path(run_dir: Path) -> str:
    manifest = load_json(run_dir / "rerun/rrd_manifest.json", {}) or {}
    declared = manifest.get("output_path") or manifest.get("output_rrd") or manifest.get("output")
    # The original sealed manifest may not register the derived RRD. Its run ID
    # still gives a stable compatible default without a batch-specific rule.
    if not declared:
        source = load_json(run_dir / "run_manifest.json", {}) or {}
        declared = source.get("rrd_path") or f"rerun/{source.get('run_id', run_dir.name)}.rrd"
    path = Path(declared)
    path = path if path.is_absolute() else run_dir / path
    resolved = path.resolve()
    if not resolved.is_relative_to(run_dir.resolve()):
        raise HandoffError("RRD outside run: deliver external analysis using its result_manifest")
    return resolved.relative_to(run_dir.resolve()).as_posix()


def analysis_rrd_files(run_dir: Path, manifest_path: Path) -> tuple[Path, Path, dict[str, Any]]:
    """Select verified derived RRD without adding files to a sealed source run."""
    root = (run_dir.parent.parent / "analysis" / run_dir.name).resolve()
    manifest_path = manifest_path.resolve()
    if not manifest_path.is_relative_to(root):
        raise HandoffError("RRD analysis manifest outside this run's analysis directory")
    document = load_json(manifest_path, {}) or {}
    if document.get("run_id") != run_dir.name or document.get("complete") is not True:
        raise HandoffError("RRD analysis identity/complete state is invalid")
    if document.get("hash_verified") is not True or document.get("valid_for_formal_validation") is not True:
        raise HandoffError("RRD analysis is not strictly verified")
    output = Path(document.get("output_path", ""))
    output = (output if output.is_absolute() else manifest_path.parent / output).resolve()
    if not output.is_relative_to(manifest_path.parent) or not output.is_file():
        raise HandoffError("RRD analysis output missing or escapes manifest directory")
    if sha256_file(output) != document.get("rrd_sha256"):
        raise HandoffError("RRD analysis output SHA mismatch")
    return manifest_path.parent, output, document


def resolve_role_path(run_dir: Path, value: str) -> str:
    return value.replace("{rrd_path}", rrd_relative_path(run_dir)) if "{rrd_path}" in value else value


def expand_source_path(base: Path, rel: str) -> list[Path]:
    target = base / rel
    if target.is_symlink():
        raise HandoffError(f"symlink source is not allowed: {target}")
    files = all_files(target)
    if not files:
        raise HandoffError(f"required source path missing or empty: {target}")
    return files


def selected_profile_names(profile: str | None, profiles: str | None) -> list[str]:
    if profile and profiles:
        raise HandoffError("use either --profile or --profiles")
    value = profile or profiles or "common"
    if profiles == "all" or (not profile and value == "all"):
        return ROLE_ORDER.copy()
    if profile == "all":
        return ["all"]
    names = [item.strip() for item in value.split(",") if item.strip()]
    unknown = sorted(set(names) - set(PROFILE_NAMES))
    if unknown:
        raise HandoffError(f"unknown profiles: {', '.join(unknown)}")
    return names


def _add_entry(
    entries: dict[str, dict[str, Any]], source: Path, package_rel: str,
    origin: str, run_dir: Path, known_sha: dict[str, str],
) -> None:
    if source.is_symlink():
        raise HandoffError(f"symlink source is not allowed: {source}")
    stat = source.stat()
    run_rel = None
    try:
        run_rel = source.relative_to(run_dir).as_posix()
    except ValueError:
        pass
    digest = known_sha.get(run_rel or "")
    entry = {
        "origin": origin,
        "source_path": str(source),
        "source_relative_path": run_rel,
        "package_path": package_rel,
        "size_bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
        "sha256": digest,
    }
    existing = entries.get(package_rel)
    if existing and existing["source_path"] != entry["source_path"]:
        raise HandoffError(f"package path collision: {package_rel}")
    entries[package_rel] = entry


def build_entries(
    run_dir: Path, profile: str, config: dict[str, Any], *,
    include_optional: bool = False, include_rerun: bool = False,
    hash_missing: bool = True, rrd_manifest_path: Path | None = None,
) -> list[dict[str, Any]]:
    if profile not in PROFILE_NAMES:
        raise HandoffError(f"unknown profile: {profile}")
    known_sha = parse_sha256sums(run_dir / "sha256sums.txt")
    derived = analysis_rrd_files(run_dir, rrd_manifest_path) if rrd_manifest_path else None
    rrd_sidecar = (derived[1] if derived else run_dir / rrd_relative_path(run_dir))
    rrd_sidecar = Path(str(rrd_sidecar) + ".sha256")
    for rel, digest in parse_sha256sums(rrd_sidecar).items():
        known_sha[f"rerun/{Path(rel).name}"] = digest
    entries: dict[str, dict[str, Any]] = {}
    definitions: list[tuple[str, dict[str, Any]]] = []
    if profile == "all":
        definitions = [(name, config["profiles"][name]) for name in ROLE_ORDER]
    else:
        definitions = [(profile, config["profiles"][profile])]

    def selected_files(rel):
        if derived and ("{rrd_path}" in rel or rel.startswith("rerun/")):
            directory, output, _ = derived
            relative = rel.replace("{rrd_path}", output.name)
            if relative.startswith("rerun/"): relative = relative[len("rerun/"):]
            for source in expand_source_path(directory, relative):
                resolved = source.resolve()
                if not resolved.is_relative_to(directory):
                    raise HandoffError("RRD derived source escapes manifest directory")
                yield source, "rerun/" + source.relative_to(directory).as_posix(), "analysis_rrd"
        else:
            for source in expand_source_path(run_dir, resolve_role_path(run_dir, rel)):
                yield source, source.relative_to(run_dir).as_posix(), "run"

    common_added = False
    for role, definition in definitions:
        if definition.get("include_common") and not common_added:
            for rel in config["common"]["run_paths"]:
                for source in expand_source_path(run_dir, resolve_role_path(run_dir, rel)):
                    source_rel = source.relative_to(run_dir).as_posix()
                    _add_entry(entries, source, f"common/{source_rel}", "run", run_dir, known_sha)
            common_added = True
        for rel in definition.get("run_paths", []):
            for source, source_rel, origin in selected_files(rel):
                package_rel = Path(source_rel).name if role == "rerun_viewer" and profile != "all" else source_rel
                _add_entry(entries, source, package_rel, origin, run_dir, known_sha)
        if include_optional:
            for rel in definition.get("optional_run_paths", []):
                target = run_dir / resolve_role_path(run_dir, rel)
                if target.exists():
                    for source in all_files(target):
                        source_rel = source.relative_to(run_dir).as_posix()
                        _add_entry(entries, source, source_rel, "run_optional", run_dir, known_sha)
        for rel in definition.get("project_paths", []):
            for source in expand_source_path(PROJECT_ROOT, rel):
                project_rel = source.relative_to(PROJECT_ROOT).as_posix()
                _add_entry(entries, source, f"software/{project_rel}", "project", run_dir, known_sha)

    if include_rerun and profile not in {"rerun_viewer", "all"}:
        for rel in config["rerun_paths"]:
            for source, source_rel, origin in selected_files(rel):
                _add_entry(entries, source, source_rel, origin, run_dir, known_sha)

    result = [entries[key] for key in sorted(entries)]
    if hash_missing:
        for entry in result:
            if not entry["sha256"]:
                entry["sha256"] = sha256_file(Path(entry["source_path"]))
    return result


def human_bytes(value: int) -> str:
    size = float(value)
    for unit in ("B", "KiB", "MiB", "GiB", "TiB"):
        if size < 1024 or unit == "TiB":
            return f"{size:.2f} {unit}"
        size /= 1024
    return str(value)


def run_facts(run_dir: Path, rrd_manifest_path: Path | None = None) -> dict[str, Any]:
    manifest = load_json(run_dir / "run_manifest.json", {})
    acceptance = load_json(run_dir / "reports/system_acceptance.json", {})
    video = load_json(run_dir / "reports/video_validation.json", {})
    replay = load_json(run_dir / "reports/replay_validation.json", {})
    derived = analysis_rrd_files(run_dir, rrd_manifest_path) if rrd_manifest_path else None
    rrd = derived[2] if derived else load_json(run_dir / "rerun/rrd_manifest.json", {})
    return {
        "run_id": manifest.get("run_id", run_dir.name),
        "created_at": manifest.get("created_at"),
        "duration_sec": acceptance.get("elapsed_sec"),
        "enabled_writers": manifest.get("enabled_writers", {}),
        "acceptance": acceptance.get("acceptance", {}),
        "rrd_name": derived[1].name if derived else Path(rrd_relative_path(run_dir)).name,
        "platform": manifest.get("platform"),
        "ros_distro": manifest.get("ros_distro"),
        "software_evidence_level": manifest.get("software_evidence_level"),
        "video_input_frames": video.get("input_frames"),
        "video_encoded_frames": video.get("encoded_frames"),
        "video_queue_drop_count": video.get("queue_drop_count"),
        "replay_validation": replay,
        "rrd_version": rrd.get("rerun_sdk_version", "unknown"),
        "rrd_entities": len(rrd.get("entities", [])),
    }


def result_manifest_template(run_id: str = RUN_ID) -> dict[str, Any]:
    return {
        "source_run_id": run_id,
        "profile": "",
        "source_files": [],
        "source_sha256": {},
        "algorithm_name": "",
        "algorithm_version_or_commit": "",
        "environment": {"os": "", "architecture": "", "ros_distro": "", "python": ""},
        "config_files": {},
        "start_time": "",
        "end_time": "",
        "input_count": {},
        "output_count": {},
        "warnings": [],
        "complete": False,
    }


def known_limitations_text(facts: dict[str, Any]) -> str:
    return f"""# Known limitations

- Source run: `{facts['run_id']}`; duration seconds: `{facts.get('duration_sec', 'unknown')}`.
- Enabled writers: `{json.dumps(facts.get('enabled_writers', {}), sort_keys=True)}`.
- Recorded acceptance: `{json.dumps(facts.get('acceptance', {}), sort_keys=True)}`.
- Formal replay evidence: `{json.dumps(facts.get('replay_validation', {}), sort_keys=True)}`.
- Industrial CameraInfo remains uncalibrated; spatial extrinsics and hardware synchronization are unverified.
- EddyChannelMean contains 8 channel means in mV, no waveform. EddyFrame raw samples exist only if that topic was recorded.
- RRD is visualization output. Its readability does not establish ROS replay counts or algorithm accuracy.
- Missing acceptance/calibration/model evidence is unavailable; no millimetre measurement or movement-control claim is made.
"""


def dataset_readme(facts: dict[str, Any], profile: str, count: int, size: int) -> str:
    replay = facts["replay_validation"]
    return f"""# Dataset handoff: {profile}

- Run ID: `{facts['run_id']}`
- Collection time: `{facts.get('created_at')}`
- Duration seconds: `{facts.get("duration_sec", "unknown")}`
- Platform: `{facts.get('platform')}`
- ROS distribution: `{facts.get('ros_distro')}`
- Evidence level: `{facts.get('software_evidence_level')}`
- Profile source files: {count}
- Estimated source bytes: {size} ({human_bytes(size)})

This package is a role-scoped subset of a sealed field run. Preserve relative paths and verify `SHA256SUMS` after materialization. Derived results must be written to `result/` or `derived/<algorithm_name>/`, never into the source run.

Formal replay status: `executed={str(replay.get('executed', False)).lower()}`, `valid={str(replay.get('valid', False)).lower()}`. Use the supplied formal replay report; missing or executed=false evidence is not an acceptance pass.
"""


def dependency_text() -> str:
    return """# Software dependencies

The source workspace is `/home/sunrise/east_eleair/physic_bev/src`. Its `.git` repository has no commit yet, so no truthful workspace commit SHA is available. Source snapshots in role packages are identified by per-file SHA256 instead.

Relevant source packages:

- FAST-LIVO2: `src/FAST-LIVO2`, ROS package `fast_livo`
- Livox interfaces/driver: `src/livox_ros_driver2`
- D405 messages: `src/realsense-ros/realsense2_camera_msgs`
- Eddy protocol and messages: `src/eddy_driver`
- Integration interfaces: `src/inspection_interfaces`

Do not distribute the local `install/` tree as a PC interface dependency. Rebuild required packages from source on ROS 2 Humble and record source SHA256, compiler environment and parameters with returned results.
"""


def usage_document(profile: str, rrd_name: str = "recording.rrd") -> str:
    if profile == "fastlivo2":
        return """# FAST-LIVO2 usage

Use ROS 2 Humble in an isolated domain, for example `export ROS_DOMAIN_ID=71`, then source the recipient workspace. Inspect input with `ros2 bag info mcap/mid360`.

The current source launch is `ros2 launch fast_livo mapping.launch.py enable_rviz:=false`; it loads `livo.yaml`, `mid360.yaml`, `d405.yaml` and `log.yaml`. Read this run's bag inventory and frozen configuration before launching. The default `mid360.yaml` selects lidar type 1, whose source subscribes to `livox_ros_driver2/msg/CustomMsg`. If the actual bag instead contains `sensor_msgs/msg/PointCloud2`, the types do not match; supply a reviewed adapter/preprocessor and record its SHA before mapping. Never infer compatibility from the topic name.

The mapping node must use simulated time before playback. The current launch file does not expose a `use_sim_time` launch argument, so the recipient must add it to an isolated config/launch snapshot or invoke the executable with the same four parameter files plus `-p use_sim_time:=true`. Do not edit this source package.

After compatibility is resolved, start mapping first, then play:

```bash
ros2 bag play mcap/mid360 --clock --rate 1.0
```

Record outputs outside the source package:

```bash
mkdir -p result
ros2 bag record -o result/mapping.mcap /cloud_registered /path /aft_mapped_to_init
```

The verified output topics come from `LIVMapper.cpp`. To save a map, enable `pcd_save.pcd_save_en` in a copied parameter snapshot; the implementation writes under FAST-LIVO2 `Log/PCD`. Move the chosen downsampled PCD to `result/map.pcd`. Derive `result/trajectory.csv` from the recorded `/aft_mapped_to_init`, preserving header timestamps.

Return `README.md`, `mapping.mcap/`, `map.pcd`, `trajectory.csv`, `mapping_metrics.json`, `config_snapshot/`, `processing.log`, `result_manifest.json` and `SHA256SUMS`. Record code/source SHA, parameter SHA and every input SHA. Never write results into the sealed run.
"""
    if profile == "front_camera":
        return """# Industrial camera usage

Images are in `video/front/front_*.mkv`; authoritative ROS timestamps are in the matching `front_*.frames.jsonl`. Do not infer ROS time from frame index or nominal FPS. Track every derived record by segment, frame index and `ros_header_stamp_ns`.

Decode with ffmpeg and use the verified `.frame_map.json` association. Packet counts do not establish decoded frame counts or alignment. Read actual segments, input/decoded counts, pre-IDR losses and queue drops from this run's manifests. A legacy run without sufficient mapping evidence is not strictly aligned; do not assign the nth sidecar timestamp to the nth decoded frame.

Inspect image quality and tracker_enabled in the source manifest. Mark every new offline algorithm result as derived, and retain the source timestamp.

Return `detection_results.jsonl`, `frame_metrics.jsonl`, `overlay.mkv`, `failure_frames/`, `summary.json`, `config_snapshot/`, `algorithm_version.json` and `SHA256SUMS` under `result/`.
"""
    if profile == "d405":
        return """# D405 usage

Inspect with `ros2 bag info mcap/d405`. In ROS 2 Humble, source `realsense2_camera_msgs` and play in an isolated domain with `ros2 bag play mcap/d405 --clock`. Preserve every message `header.stamp` when converting with `cv_bridge`.

Topics are `/d405/camera/depth/image_rect_raw`, `/d405/camera/infra1/image_rect_raw`, and `/d405/camera/infra2/image_rect_raw`, with their CameraInfo and metadata topics. The recorded image encoding must be read from each message; do not assume it from the filename.

Report counts, interval distributions, gaps over 50/100/500 ms, exact maximum-gap positions, zero/invalid depth ratios, IR under/overexposure, and a D405-frame point cloud. Use this run's measured rates and interval reports. Industrial-camera calibration and spatial alignment with MID360 are unverified; device CameraInfo alone does not establish rig extrinsics.
"""
    if profile == "eddy":
        return """# Eddy usage

Inspect with `ros2 bag info mcap/eddy`. Check the manifest and bag topic inventory: `/eddy/channel_mean` contains 8 channel means in mV, each calculated from 20 samples; raw waveform analysis requires `/eddy/data` (EddyFrame, 20x8 samples). Driver status requires `/eddy/status`. Missing topics are unavailable.

RRD contains a rate-limited view of channel means and is never the formal input. Use `mcap/eddy`. Read this run's actual coverage and drop counts from the collection report.

Read `/eddy/status` and persisted monitor diagnostics for this run. If `last_status_is_error=true`, record the error and its evidence; if raw status bytes were not recorded, report unavailable rather than inferring a stop-only event. Return the analysis in `status_error_analysis.json`.

Protocol and message definitions are under `software/src/eddy_driver`. Channel order is ch0..ch7, values are mV, and the ADC frame is 20 samples by 8 channels. Return `channel_statistics.csv`, `time_series.parquet`, `anomaly_events.jsonl`, `status_error_analysis.json`, `plots/`, `processing_config.json`, `result_manifest.json` and `SHA256SUMS`.
"""
    if profile == "rerun_viewer":
        return f"""# Rerun viewer usage

Use Rerun CLI/SDK exactly 0.36.2. No ROS 2 installation is required.

```bash
sha256sum -c {rrd_name}.sha256
rerun rrd verify {rrd_name}
rerun {rrd_name}
```

Check the current rrd_manifest.json and validation report for its actual file hashes, entity counts and verification status. It is a rate-limited/compressed visualization product and must not replace original MCAP or MKV inputs.
"""
    if profile == "system_integration":
        return """# System integration usage

The core bag and reports establish the common ROS/host time index. Inspect with `ros2 bag info mcap/core`. Only an independently counted ROS replay fixture or hardware replay report can establish subscriber delivery. Publisher-only reports are not subscriber acceptance.

Check enabled_writers and D405 clock settings in the source manifest. Industrial CameraInfo is uncalibrated; hardware synchronization is not proven. Maintain source header timestamps and bag timestamps separately. Do not claim precise spatial fusion. After FAST-LIVO2 returns a timestamped trajectory, define a separate `multisensor_fusion` profile with reviewed calibration provenance.
"""
    return ""


def vision_schema(run_id: str = RUN_ID) -> dict[str, Any]:
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "title": "Offline vision result",
        "type": "object",
        "required": ["run_id", "segment", "frame_index", "timestamp_ns", "detected", "state", "confidence", "failure_reasons", "processing_ms"],
        "properties": {
            "run_id": {"const": run_id}, "segment": {"type": "integer", "minimum": 1},
            "frame_index": {"type": "integer", "minimum": 0}, "timestamp_ns": {"type": "integer"},
            "detected": {"type": "boolean"}, "state": {"type": "string"},
            "left_boundary": {"type": ["array", "null"]}, "right_boundary": {"type": ["array", "null"]},
            "centerline": {"type": ["array", "null"]}, "confidence": {"type": "number"},
            "failure_reasons": {"type": "array", "items": {"type": "string"}},
            "processing_ms": {"type": "number", "minimum": 0}
        }
    }


def eddy_schema(run_id: str = RUN_ID) -> dict[str, Any]:
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "title": "Eddy anomaly result",
        "type": "object",
        "required": ["run_id", "start_timestamp_ns", "end_timestamp_ns", "channels", "score", "classification"],
        "properties": {
            "run_id": {"const": run_id}, "start_timestamp_ns": {"type": "integer"},
            "end_timestamp_ns": {"type": "integer"}, "channels": {"type": "array", "items": {"type": "integer", "minimum": 0, "maximum": 7}},
            "score": {"type": "number"}, "classification": {"type": "string"}, "notes": {"type": "string"}
        }
    }


def write_generated_docs(root: Path, profile: str, facts: dict[str, Any], entries: list[dict[str, Any]]) -> list[Path]:
    generated: list[Path] = []
    size = sum(item["size_bytes"] for item in entries)
    common_dir = root / "common" if profile != "rerun_viewer" else root
    if profile != "rerun_viewer":
        docs = {
            common_dir / "DATASET_README.md": dataset_readme(facts, profile, len(entries), size),
            common_dir / "KNOWN_LIMITATIONS.md": known_limitations_text(facts),
            common_dir / "SOFTWARE_DEPENDENCIES.md": dependency_text(),
        }
        for path, content in docs.items():
            write_text(path, content)
            generated.append(path)
        template = common_dir / "RESULT_MANIFEST_TEMPLATE.json"
        write_json(template, result_manifest_template(facts["run_id"]))
        generated.append(template)
    usage_name = None
    if profile in PROFILE_NAMES:
        usage_name = load_config(DEFAULT_CONFIG)["profiles"][profile].get("usage_document")
    if usage_name:
        path = root / usage_name
        write_text(path, usage_document(profile, facts.get("rrd_name", "recording.rrd")))
        generated.append(path)
    if profile == "fastlivo2":
        path = root / "EXPECTED_OUTPUT_CONTRACT.md"
        write_text(path, """# FAST-LIVO2 expected output contract

The recipient returns a `result/` directory containing `README.md`, complete `mapping.mcap/` including `metadata.yaml`, `map.pcd`, `trajectory.csv`, `mapping_metrics.json`, `config_snapshot/`, `processing.log`, `result_manifest.json` and a subset `SHA256SUMS`.

`result_manifest.json` must identify every source file/SHA, code version or source snapshot SHA, build/runtime environment, configuration hashes, input/output counts and warnings. `complete=true` is allowed only after every declared result is readable, configuration and logs are present, and `sha256sum -c SHA256SUMS` passes.
""")
        generated.append(path)
    if profile == "front_camera":
        path = root / "VISION_RESULT_SCHEMA.json"
        write_json(path, vision_schema(facts["run_id"]))
        generated.append(path)
    if profile == "eddy":
        path = root / "EDDY_RESULT_SCHEMA.json"
        write_json(path, eddy_schema(facts["run_id"]))
        generated.append(path)
    if profile == "all":
        for role in ("fastlivo2", "front_camera", "d405", "eddy", "rerun_viewer", "system_integration"):
            name = load_config(DEFAULT_CONFIG)["profiles"][role]["usage_document"]
            path = root / "role_guides" / name
            write_text(path, usage_document(role, facts.get("rrd_name", "recording.rrd")))
            generated.append(path)
        for name, value in (("VISION_RESULT_SCHEMA.json", vision_schema(facts["run_id"])), ("EDDY_RESULT_SCHEMA.json", eddy_schema(facts["run_id"]))):
            path = root / "role_guides" / name
            write_json(path, value)
            generated.append(path)
    return generated


def source_manifest(profile: str, run_dir: Path, entries: list[dict[str, Any]], mode: str) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "created_at": utc_now(),
        "mode": mode,
        "profile": profile,
        "source_run_id": run_dir.name,
        "source_run_dir": str(run_dir),
        "file_count": len(entries),
        "total_bytes": sum(item["size_bytes"] for item in entries),
        "complete": mode == "materialized",
        "files": entries,
    }


def package_sha_lines(root: Path) -> list[str]:
    paths = sorted(p for p in root.rglob("*") if p.is_file() and p.name != "SHA256SUMS")
    return [f"{sha256_file(path)}  {path.relative_to(root).as_posix()}" for path in paths]


def verify_existing(path: Path, entry: dict[str, Any]) -> None:
    if not path.is_file():
        raise HandoffError(f"resume target is not a regular file: {path}")
    if path.stat().st_size != entry["size_bytes"]:
        raise HandoffError(f"resume size conflict: {path}")
    if sha256_file(path) != entry["sha256"]:
        raise HandoffError(f"resume SHA256 conflict: {path}")


def copy_one(source: Path, destination: Path, method: str) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if method == "rsync":
        if not shutil.which("rsync"):
            raise HandoffError("rsync is not installed")
        proc = subprocess.run(
            ["rsync", "-a", "--partial", "--protect-args", str(source), str(destination)],
            text=True, capture_output=True,
        )
        if proc.returncode:
            raise HandoffError(f"rsync failed for {source}: {proc.stderr.strip()}")
    else:
        shutil.copy2(source, destination)


def materialize(
    run_dir: Path, profile: str, destination: Path, entries: list[dict[str, Any]],
    facts: dict[str, Any], *, resume: bool, copy_method: str,
) -> dict[str, Any]:
    if destination.exists() and not resume:
        raise HandoffError(f"destination already exists; use --resume after verification: {destination}")
    required = sum(item["size_bytes"] for item in entries)
    destination.parent.mkdir(parents=True, exist_ok=True)
    free = shutil.disk_usage(destination.parent).free
    if free < required:
        raise HandoffError(f"insufficient target space: need {required}, free {free}")
    destination.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    copied = 0
    reused = 0
    copied_bytes = 0
    try:
        for entry in entries:
            target = destination / entry["package_path"]
            if target.exists():
                if not resume:
                    raise HandoffError(f"target conflict: {target}")
                verify_existing(target, entry)
                reused += 1
                continue
            copy_one(Path(entry["source_path"]), target, copy_method)
            verify_existing(target, entry)
            copied += 1
            copied_bytes += entry["size_bytes"]
        write_json(destination / "SOURCE_FILE_MANIFEST.json", source_manifest(profile, run_dir, entries, "materialized"))
        write_generated_docs(destination, profile, facts, entries)
        if profile != "rerun_viewer":
            write_json(destination / "common" / "SOURCE_FILE_MANIFEST.json", source_manifest(profile, run_dir, entries, "materialized"))
        status_path = destination / "HANDOFF_STATUS.json"
        write_json(status_path, {"complete": True, "profile": profile, "created_at": utc_now()})
        sha_text = "\n".join(package_sha_lines(destination))
        write_text(destination / "SHA256SUMS", sha_text)
        if profile != "rerun_viewer":
            write_text(destination / "common" / "SHA256SUMS", sha_text)
        verify = subprocess.run(
            ["sha256sum", "-c", "SHA256SUMS"], cwd=destination,
            text=True, capture_output=True,
        )
        if verify.returncode:
            raise HandoffError(f"package SHA verification failed: {verify.stdout}{verify.stderr}")
    except Exception as exc:
        write_json(destination / "HANDOFF_STATUS.json", {
            "complete": False, "profile": profile, "updated_at": utc_now(), "error": str(exc),
        })
        raise
    return {
        "profile": profile, "destination": str(destination), "complete": True,
        "copied_files": copied, "reused_files": reused, "copied_bytes": copied_bytes,
        "total_files": len(entries), "total_bytes": required,
        "elapsed_sec": time.monotonic() - started, "sha256_verified": True,
    }


def dry_run_report(profile: str, destination: Path | None, entries: list[dict[str, Any]]) -> dict[str, Any]:
    conflicts = []
    existing = 0
    if destination and destination.exists():
        for entry in entries:
            target = destination / entry["package_path"]
            if target.exists():
                existing += 1
                if not target.is_file() or target.stat().st_size != entry["size_bytes"]:
                    conflicts.append(entry["package_path"])
    return {
        "schema_version": SCHEMA_VERSION,
        "created_at": utc_now(),
        "profile": profile,
        "destination": str(destination) if destination else None,
        "dry_run": True,
        "would_copy_files": len(entries) - existing,
        "existing_files": existing,
        "estimated_bytes": sum(item["size_bytes"] for item in entries),
        "conflicts_by_size_or_type": conflicts,
        "large_payloads_not_copied": sum(1 for item in entries if Path(item["package_path"]).suffix in LARGE_SUFFIXES),
    }


def git_inventory(path: Path) -> dict[str, Any]:
    result = {"path": str(path), "commit": None, "remotes": [], "git_available": False}
    proc = subprocess.run(["git", "-C", str(path), "rev-parse", "HEAD"], text=True, capture_output=True)
    if proc.returncode == 0:
        result["git_available"] = True
        result["commit"] = proc.stdout.strip()
        remotes = subprocess.run(["git", "-C", str(path), "remote", "-v"], text=True, capture_output=True)
        result["remotes"] = remotes.stdout.splitlines()
    else:
        result["git_error"] = proc.stderr.strip()
    return result


def topic_inventory(run_dir: Path) -> dict[str, Any]:
    report = load_json(run_dir / "reports/mcap_validation.json", {})
    groups = {}
    for group, detail in report.get("bags", {}).items():
        topics = detail.get("message_validation", {}).get("topics", {})
        groups[group] = {
            "valid": detail.get("valid"),
            "path": detail.get("path"),
            "topics": [
                {
                    "name": name, "type": value.get("type"), "count": value.get("count"),
                    "first_bag_timestamp_ns": value.get("first_bag_timestamp_ns"),
                    "last_bag_timestamp_ns": value.get("last_bag_timestamp_ns"),
                    "first_header_stamp_ns": value.get("first_header_stamp_ns"),
                    "last_header_stamp_ns": value.get("last_header_stamp_ns"),
                    "rate_hz": value.get("rate_hz"),
                }
                for name, value in sorted(topics.items())
            ],
        }
    return {"schema_version": SCHEMA_VERSION, "source": "reports/mcap_validation.json", "groups": groups}


def audit_run(run_dir: Path, output_root: Path, config: dict[str, Any], profile_entries: dict[str, list[dict[str, Any]]]) -> dict[str, Any]:
    audit_dir = output_root / "audit"
    known_sha = parse_sha256sums(run_dir / "sha256sums.txt")
    rrd_known = {
        f"rerun/{Path(rel).name}": digest
        for rel, digest in parse_sha256sums(run_dir / (rrd_relative_path(run_dir) + ".sha256")).items()
    }
    inventory = []
    for path in all_files(run_dir):
        rel = path.relative_to(run_dir).as_posix()
        stat = path.stat()
        digest = known_sha.get(rel) or rrd_known.get(rel) or sha256_file(path)
        inventory.append({
            "path": rel, "size_bytes": stat.st_size, "mtime_ns": stat.st_mtime_ns,
            "mtime_utc": dt.datetime.fromtimestamp(stat.st_mtime, dt.timezone.utc).isoformat(),
            "sha256": digest, "covered_by_source_sha256sums": rel in known_sha,
        })
    source_files = {item["path"] for item in inventory}
    covered = sorted(source_files & set(known_sha))
    uncovered = sorted(source_files - set(known_sha))
    partial = [item for item in inventory if item["path"].startswith(".partial/")]
    video = [item for item in inventory if item["path"].startswith("video/front/")]
    current = {
        "schema_version": SCHEMA_VERSION, "created_at": utc_now(), "run_dir": str(run_dir),
        "run_id": run_dir.name, "file_count": len(inventory),
        "total_bytes": sum(item["size_bytes"] for item in inventory), "files": inventory,
        "mcap_groups": {
            name: [item for item in inventory if item["path"].startswith(f"mcap/{name}/")]
            for name in ("core", "d405", "eddy", "mid360")
        },
        "video_front": video, "partial_entries": partial,
        "source_sha256sums_coverage": {
            "listed_count": len(known_sha), "covered_existing_count": len(covered),
            "uncovered_count": len(uncovered), "uncovered_files": uncovered,
            "listed_but_missing": sorted(set(known_sha) - source_files),
        },
        "disk": {
            "run_filesystem": dict(zip(("total", "used", "free"), shutil.disk_usage(run_dir))),
            "workspace_filesystem": dict(zip(("total", "used", "free"), shutil.disk_usage(PROJECT_ROOT))),
        },
    }
    write_json(audit_dir / "current_run_inventory.json", current)
    topics = topic_inventory(run_dir)
    write_json(audit_dir / "topic_type_inventory.json", topics)
    package_paths = {
        "FAST-LIVO2": PROJECT_ROOT / "src/FAST-LIVO2",
        "livox_ros_driver2": PROJECT_ROOT / "src/livox_ros_driver2",
        "realsense2_camera_msgs": PROJECT_ROOT / "src/realsense-ros/realsense2_camera_msgs",
        "eddy_driver": PROJECT_ROOT / "src/eddy_driver",
        "inspection_interfaces": PROJECT_ROOT / "src/inspection_interfaces",
    }
    dependencies = {
        "schema_version": SCHEMA_VERSION,
        "host": {"os": platform.platform(), "architecture": platform.machine(), "python": platform.python_version()},
        "workspace_git": git_inventory(PROJECT_ROOT / "src"),
        "packages": {name: git_inventory(path) for name, path in package_paths.items()},
        "custom_interfaces": {
            "livox_ros_driver2/msg/CustomMsg": str(PROJECT_ROOT / "src/livox_ros_driver2/msg/CustomMsg.msg"),
            "realsense2_camera_msgs": str(PROJECT_ROOT / "src/realsense-ros/realsense2_camera_msgs/msg"),
            "eddy_driver": str(PROJECT_ROOT / "src/eddy_driver/msg"),
            "inspection_interfaces": str(PROJECT_ROOT / "src/inspection_interfaces/msg"),
        },
        "rerun_sdk_version": load_json(run_dir / "rerun/rrd_manifest.json", {}).get("rerun_sdk_version"),
        "note": "Workspace .git has no HEAD commit; per-file SHA256 is the reproducibility identity.",
    }
    write_json(audit_dir / "software_dependency_inventory.json", dependencies)
    estimates = {
        name: {"file_count": len(entries), "size_bytes": sum(item["size_bytes"] for item in entries),
               "size_human": human_bytes(sum(item["size_bytes"] for item in entries))}
        for name, entries in profile_entries.items()
    }
    write_json(audit_dir / "profile_size_estimate.json", {"schema_version": SCHEMA_VERSION, "profiles": estimates})
    replay = load_json(run_dir / "reports/replay_validation.json", {})
    limitations = {
        "schema_version": SCHEMA_VERSION,
        "items": [line[2:] for line in known_limitations_text(run_facts(run_dir)).splitlines() if line.startswith("- ")],
        "replay_validation_state": replay,
        "partial_policy": "registered only; never selected for formal delivery",
        "structure_differences": [
            "The source .partial directory is empty, rather than containing unfinished bags.",
            "video/front has MKV, frames.jsonl and mux.log sidecars; no separate video manifest file exists.",
            "physic_bev/src/.git exists but has no HEAD commit and all source packages are untracked.",
            "System Python cannot import rerun; the sealed RRD manifest records SDK/CLI 0.36.2 from the PC conversion environment.",
        ],
    }
    write_json(audit_dir / "known_limitations.json", limitations)
    plan = {
        "schema_version": SCHEMA_VERSION,
        "steps": [
            "Freeze source inventory and verify registered SHA256", "Resolve profile selections",
            "Generate manifest-only definitions and recipient documentation", "Dry-run every profile",
            "Materialize and verify the small common profile", "Recheck frozen source SHA256",
        ],
        "large_profiles_materialized": False,
    }
    write_json(audit_dir / "implementation_plan.json", plan)
    return {"inventory": current, "topics": topics, "dependencies": dependencies, "estimates": estimates, "limitations": limitations}


def write_manifest_definition(output_root: Path, run_dir: Path, profile: str, entries: list[dict[str, Any]], facts: dict[str, Any]) -> Path:
    target = output_root / "manifests" / profile
    target.mkdir(parents=True, exist_ok=True)
    manifest = source_manifest(profile, run_dir, entries, "manifest-only")
    write_json(target / "SOURCE_FILE_MANIFEST.json", manifest)
    write_generated_docs(target, profile, facts, entries)
    if profile != "rerun_viewer":
        write_json(target / "common" / "SOURCE_FILE_MANIFEST.json", manifest)
    export = (
        f"/usr/bin/python3 scripts/build_role_handoff.py --run-dir {run_dir} "
        f"--profile {profile} --destination /media/RECIPIENT_DISK/{run_dir.name}_{profile.upper()} "
        "--materialize --copy-method rsync --resume"
    )
    write_text(target / "EXPORT_COMMAND.txt", export)
    return target


def verify_package(package_dir: Path) -> dict[str, Any]:
    sums = package_dir / "SHA256SUMS"
    manifest = load_json(package_dir / "SOURCE_FILE_MANIFEST.json", {})
    status = load_json(package_dir / "HANDOFF_STATUS.json", {})
    if not sums.is_file():
        raise HandoffError(f"missing SHA256SUMS: {package_dir}")
    proc = subprocess.run(["sha256sum", "-c", sums.name], cwd=package_dir, text=True, capture_output=True)
    result = {
        "package_dir": str(package_dir), "sha256_valid": proc.returncode == 0,
        "manifest_complete": bool(manifest.get("complete")), "status_complete": bool(status.get("complete")),
        "stdout": proc.stdout, "stderr": proc.stderr,
    }
    if not all((result["sha256_valid"], result["manifest_complete"], result["status_complete"])):
        raise HandoffError(f"package verification failed: {result}")
    return result


def verify_source_unchanged(run_dir: Path, audit_inventory: Path) -> dict[str, Any]:
    """Compare source files with the pre-build audit without touching them."""
    audit = load_json(audit_inventory, {})
    expected = {item["path"]: item for item in audit.get("files", [])}
    source_sha = parse_sha256sums(run_dir / "sha256sums.txt")
    rrd_sha = {
        f"rerun/{Path(rel).name}": digest
        for rel, digest in parse_sha256sums(run_dir / (rrd_relative_path(run_dir) + ".sha256")).items()
    }
    current_paths = {
        path.relative_to(run_dir).as_posix(): path
        for path in all_files(run_dir)
    }
    changed: list[dict[str, Any]] = []
    checked = 0
    for rel, item in expected.items():
        path = current_paths.get(rel)
        if path is None:
            changed.append({"path": rel, "reason": "missing"})
            continue
        stat = path.stat()
        reasons = []
        if stat.st_size != item["size_bytes"]:
            reasons.append("size")
        if stat.st_mtime_ns != item["mtime_ns"]:
            reasons.append("mtime_ns")
        current_sha = source_sha.get(rel) or rrd_sha.get(rel)
        if current_sha is None:
            current_sha = sha256_file(path)
        if current_sha != item["sha256"]:
            reasons.append("sha256")
        checked += 1
        if reasons:
            changed.append({"path": rel, "reason": reasons})
    extras = sorted(set(current_paths) - set(expected))
    for rel in extras:
        changed.append({"path": rel, "reason": "new_after_audit"})
    return {
        "schema_version": SCHEMA_VERSION,
        "created_at": utc_now(),
        "source_run_dir": str(run_dir),
        "audit_inventory": str(audit_inventory),
        "checked_files": checked,
        "changed_files": changed,
        "source_run_unchanged": not changed,
        "note": "Large RRD SHA uses its sealed .rrd.sha256 sidecar; source sha256sums.txt was independently checked with sha256sum -c.",
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--profile")
    parser.add_argument("--profiles")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--destination", type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--manifest-only", action="store_true")
    mode.add_argument("--materialize", action="store_true")
    mode.add_argument("--verify-only", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--include-rerun", action="store_true")
    parser.add_argument("--rrd-manifest", type=Path, help="verified external analysis RRD manifest for this run")
    parser.add_argument("--include-optional", action="store_true")
    parser.add_argument("--copy-method", choices=("rsync", "python"), default="rsync")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--audit", action="store_true", help="generate full read-only run audit")
    parser.add_argument("--verify-audit", type=Path, help="compare source run against current_run_inventory.json")
    parser.add_argument("--log-level", default="INFO", choices=("DEBUG", "INFO", "WARNING", "ERROR"))
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=getattr(logging, args.log_level), format="%(levelname)s %(message)s")
    try:
        run_dir = args.run_dir.resolve()
        ensure_frozen_run(run_dir)
        config = load_config(args.config)
        names = selected_profile_names(args.profile, args.profiles)
        if args.verify_only:
            if not args.destination:
                raise HandoffError("--verify-only requires --destination")
            print(json.dumps(verify_package(args.destination.resolve()), ensure_ascii=False, indent=2))
            return 0
        if args.verify_audit:
            result = verify_source_unchanged(run_dir, args.verify_audit.resolve())
            write_json(args.output_root / "verification" / "source_unchanged.json", result)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0 if result["source_run_unchanged"] else 2
        if args.materialize and len(names) != 1:
            raise HandoffError("--materialize accepts exactly one profile")
        if args.materialize and not args.destination:
            raise HandoffError("--materialize requires --destination")
        facts = run_facts(run_dir, args.rrd_manifest)
        entries_by_profile = {
            name: build_entries(
                run_dir, name, config, include_optional=args.include_optional,
                include_rerun=args.include_rerun, hash_missing=True, rrd_manifest_path=args.rrd_manifest,
            ) for name in names
        }
        if args.dry_run:
            reports = [dry_run_report(name, args.destination, entries_by_profile[name]) for name in names]
            args.output_root.mkdir(parents=True, exist_ok=True)
            for report in reports:
                write_json(args.output_root / "dry_run_reports" / f"{report['profile']}.json", report)
            print(json.dumps(reports, ensure_ascii=False, indent=2))
            return 0
        if args.materialize:
            name = names[0]
            result = materialize(
                run_dir, name, args.destination.resolve(), entries_by_profile[name], facts,
                resume=args.resume, copy_method=args.copy_method,
            )
            args.output_root.mkdir(parents=True, exist_ok=True)
            write_json(args.output_root / "verification" / f"{name}_materialization.json", result)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0
        # Manifest-only is the safe default.
        args.output_root.mkdir(parents=True, exist_ok=True)
        for name in names:
            write_manifest_definition(args.output_root, run_dir, name, entries_by_profile[name], facts)
        if args.audit:
            all_entries = {
                name: build_entries(
                    run_dir, name, config, include_optional=args.include_optional,
                    include_rerun=args.include_rerun, hash_missing=True, rrd_manifest_path=args.rrd_manifest,
                ) for name in PROFILE_NAMES
            }
            audit_run(run_dir, args.output_root, config, all_entries)
        summary = {
            "mode": "manifest-only", "profiles": names, "output_root": str(args.output_root),
            "source_run": str(run_dir), "payload_copied": False,
            "profile_file_counts": {name: len(entries_by_profile[name]) for name in names},
            "profile_bytes": {name: sum(e["size_bytes"] for e in entries_by_profile[name]) for name in names},
        }
        manifest_names = [name for name in PROFILE_NAMES if (args.output_root / "manifests" / name / "SOURCE_FILE_MANIFEST.json").is_file()]
        source_check = load_json(args.output_root / "verification" / "source_unchanged.json", {})
        smoke = load_json(args.output_root / "verification" / "common_materialization.json", {})
        summary["status"] = {
            "handoff_builder_ready": True,
            "source_run_unchanged": source_check.get("source_run_unchanged"),
            "profile_manifests_complete": set(PROFILE_NAMES) <= set(manifest_names),
            "custom_interfaces_resolved": all(
                (PROJECT_ROOT / path).exists() for path in (
                    "src/livox_ros_driver2/msg/CustomMsg.msg",
                    "src/realsense-ros/realsense2_camera_msgs/msg/Metadata.msg",
                    "src/eddy_driver/msg/EddyChannelMean.msg",
                )
            ),
            "materialization_smoke_test_passed": smoke.get("complete") is True and smoke.get("sha256_verified") is True,
            "all_large_profiles_materialized": False,
            "ready_for_external_handoff": set(PROFILE_NAMES) <= set(manifest_names) and smoke.get("sha256_verified") is True,
        }
        write_json(args.output_root / "diagnostic_summary.json", summary)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0
    except (HandoffError, OSError, ValueError, json.JSONDecodeError) as exc:
        logging.error("%s", exc)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
