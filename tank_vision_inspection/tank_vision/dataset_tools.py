"""Dataset validation and leakage-free grouped splitting."""

from __future__ import annotations

import json
from pathlib import Path
import random
from typing import Any, Iterable

import cv2
import yaml


REQUIRED_FIELDS = (
    "sample_id",
    "session_id",
    "tank_id",
    "scan_id",
    "image_path",
    "timestamp_ns",
    "frame_id",
    "camera_serial",
    "image_width",
    "image_height",
    "pixel_format",
    "calibration_version",
    "quality_flags",
)


def load_manifest(path: str | Path) -> list[dict[str, Any]]:
    manifest_path = Path(path)
    if not manifest_path.is_file():
        raise FileNotFoundError(f"Manifest not found: {manifest_path}")
    if manifest_path.suffix.lower() == ".jsonl":
        records = [json.loads(line) for line in manifest_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    else:
        loaded = json.loads(manifest_path.read_text(encoding="utf-8"))
        records = loaded.get("samples", []) if isinstance(loaded, dict) else loaded
    if not isinstance(records, list) or not all(isinstance(item, dict) for item in records):
        raise ValueError("Manifest must contain a JSON array or JSONL objects")
    return records


def load_label_names(path: str | Path) -> set[str]:
    loaded = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    labels = loaded.get("labels", [])
    names = {item["name"] if isinstance(item, dict) else str(item) for item in labels}
    if not names:
        raise ValueError("Label specification contains no labels")
    return names


def group_key(record: dict[str, Any]) -> tuple[str, str, str]:
    return (str(record.get("session_id", "")), str(record.get("tank_id", "")), str(record.get("scan_id", "")))


def _annotation_labels(path: Path) -> list[str]:
    if path.suffix.lower() in {".yaml", ".yml"}:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    else:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    objects = loaded.get("objects", []) if isinstance(loaded, dict) else []
    return [str(item.get("label", "")) for item in objects if isinstance(item, dict)]


def validate_records(
    records: Iterable[dict[str, Any]],
    dataset_root: str | Path,
    labels_path: str | Path,
    require_files: bool = True,
) -> list[str]:
    root = Path(dataset_root)
    allowed_labels = load_label_names(labels_path)
    errors: list[str] = []
    seen_ids: set[str] = set()
    for index, record in enumerate(records):
        prefix = f"record[{index}]"
        missing = [name for name in REQUIRED_FIELDS if name not in record]
        if missing:
            errors.append(f"{prefix}: missing fields {missing}")
            continue
        sample_id = str(record["sample_id"])
        if not sample_id:
            errors.append(f"{prefix}: sample_id is empty")
        elif sample_id in seen_ids:
            errors.append(f"{prefix}: duplicate sample_id {sample_id}")
        seen_ids.add(sample_id)
        if any(not part for part in group_key(record)):
            errors.append(f"{prefix}: session_id, tank_id, and scan_id must be non-empty")
        timestamp = record["timestamp_ns"]
        if not isinstance(timestamp, int) or isinstance(timestamp, bool) or timestamp < 0:
            errors.append(f"{prefix}: timestamp_ns must be a non-negative integer")
        width, height = record["image_width"], record["image_height"]
        if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
            errors.append(f"{prefix}: image dimensions must be positive integers")
        if not isinstance(record["quality_flags"], list):
            errors.append(f"{prefix}: quality_flags must be a list")
        image_path = root / str(record["image_path"])
        if require_files and not image_path.is_file():
            errors.append(f"{prefix}: image path does not exist: {image_path}")
        elif require_files:
            image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
            if image is None:
                errors.append(f"{prefix}: image cannot be decoded: {image_path}")
            elif image.shape[1] != width or image.shape[0] != height:
                errors.append(
                    f"{prefix}: manifest dimensions {width}x{height} do not match image {image.shape[1]}x{image.shape[0]}"
                )
        calibration_version = str(record["calibration_version"] or "")
        if not calibration_version:
            errors.append(f"{prefix}: calibration_version must be non-empty")
        elif require_files and not any(
            (root / "calibration" / f"{calibration_version}{suffix}").is_file()
            for suffix in (".yaml", ".yml", ".json")
        ):
            errors.append(f"{prefix}: calibration version does not exist: {calibration_version}")
        annotation_value = record.get("annotation_path")
        if annotation_value:
            annotation_path = root / str(annotation_value)
            if require_files and not annotation_path.is_file():
                errors.append(f"{prefix}: annotation path does not exist: {annotation_path}")
            elif require_files:
                try:
                    unknown = set(_annotation_labels(annotation_path)) - allowed_labels
                    if unknown:
                        errors.append(f"{prefix}: unknown annotation labels {sorted(unknown)}")
                except (OSError, ValueError, json.JSONDecodeError, yaml.YAMLError) as exc:
                    errors.append(f"{prefix}: annotation cannot be parsed: {type(exc).__name__}")
    return errors


def create_grouped_split(
    records: Iterable[dict[str, Any]],
    train_ratio: float = 0.7,
    val_ratio: float = 0.15,
    seed: int = 42,
) -> dict[str, list[str]]:
    if train_ratio <= 0 or val_ratio < 0 or train_ratio + val_ratio >= 1:
        raise ValueError("ratios must satisfy train > 0, val >= 0, and train + val < 1")
    groups: dict[tuple[str, str, str], list[str]] = {}
    for record in records:
        key = group_key(record)
        if any(not part for part in key):
            raise ValueError("Every split record requires non-empty session_id, tank_id, and scan_id")
        groups.setdefault(key, []).append(str(record["sample_id"]))
    keys = sorted(groups)
    random.Random(seed).shuffle(keys)
    count = len(keys)
    train_end = round(count * train_ratio)
    val_end = train_end + round(count * val_ratio)
    assigned = {"train": keys[:train_end], "val": keys[train_end:val_end], "test": keys[val_end:]}
    return {name: [sample_id for key in split_keys for sample_id in groups[key]] for name, split_keys in assigned.items()}


def split_leakage(records: Iterable[dict[str, Any]], split: dict[str, list[str]]) -> list[str]:
    by_id = {str(record["sample_id"]): group_key(record) for record in records}
    owners: dict[tuple[str, str, str], str] = {}
    errors: list[str] = []
    seen_samples: set[str] = set()
    for split_name in ("train", "val", "test"):
        for sample_id in split.get(split_name, []):
            if sample_id not in by_id:
                errors.append(f"unknown sample in {split_name}: {sample_id}")
                continue
            if sample_id in seen_samples:
                errors.append(f"sample appears in multiple splits: {sample_id}")
            seen_samples.add(sample_id)
            key = by_id[sample_id]
            previous = owners.setdefault(key, split_name)
            if previous != split_name:
                errors.append(f"group leakage {key}: {previous} and {split_name}")
    return errors

