from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from tank_vision.dataset_tools import create_grouped_split, split_leakage, validate_records


ROOT = Path(__file__).parents[1]
LABELS = ROOT / "dataset" / "schema" / "labels.yaml"


def _record(sample_id: str, group: int = 0) -> dict[str, object]:
    return {
        "sample_id": sample_id,
        "session_id": f"session-{group}",
        "tank_id": f"tank-{group}",
        "scan_id": f"scan-{group}",
        "image_path": f"raw/{sample_id}.png",
        "timestamp_ns": group + 1,
        "frame_id": "camera",
        "camera_serial": "serial",
        "image_width": 16,
        "image_height": 12,
        "pixel_format": "bgr8",
        "calibration_version": "cal-v1",
        "annotation_path": f"annotations/{sample_id}.json",
        "quality_flags": [],
    }


def test_manifest_schema_is_valid_json_and_has_group_fields() -> None:
    schema = json.loads((ROOT / "dataset" / "schema" / "manifest.schema.json").read_text(encoding="utf-8"))
    assert schema["$schema"].endswith("2020-12/schema")
    assert {"session_id", "tank_id", "scan_id"}.issubset(schema["required"])


def test_dataset_path_label_dimensions_and_calibration_validation(tmp_path: Path) -> None:
    for directory in ("raw", "annotations", "calibration"):
        (tmp_path / directory).mkdir()
    record = _record("sample-1")
    assert cv2.imwrite(str(tmp_path / str(record["image_path"])), np.zeros((12, 16, 3), dtype=np.uint8))
    (tmp_path / str(record["annotation_path"])).write_text(
        json.dumps({"objects": [{"label": "surface_anomaly"}]}), encoding="utf-8"
    )
    (tmp_path / "calibration" / "cal-v1.yaml").write_text("version: cal-v1\n", encoding="utf-8")
    assert validate_records([record], tmp_path, LABELS) == []


def test_invalid_timestamp_is_reported_without_files() -> None:
    record = _record("bad-time")
    record["timestamp_ns"] = -1
    errors = validate_records([record], ROOT / "dataset", LABELS, require_files=False)
    assert any("timestamp_ns" in error for error in errors)


def test_grouped_split_has_no_session_tank_scan_leakage() -> None:
    records = [_record(f"sample-{group}-{index}", group) for group in range(6) for index in range(3)]
    split = create_grouped_split(records, 0.5, 0.25, seed=7)
    assert split_leakage(records, split) == []
    assert set(split) == {"train", "val", "test"}
    assert sum(len(items) for items in split.values()) == len(records)


def test_group_leakage_is_detected() -> None:
    records = [_record("a", 1), _record("b", 1)]
    errors = split_leakage(records, {"train": ["a"], "val": ["b"], "test": []})
    assert any("group leakage" in error for error in errors)
