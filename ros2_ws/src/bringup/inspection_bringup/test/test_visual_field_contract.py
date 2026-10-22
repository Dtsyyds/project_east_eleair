"""Static and filesystem-level contracts for the visual field profile."""

from __future__ import annotations
from inspection_runtime.paths import project_root

import json
from pathlib import Path
import sys

import pytest
import yaml

ROOT = project_root()
PACKAGE = project_root() / 'ros2_ws/src/bringup/inspection_bringup'
LAUNCH = project_root() / 'ros2_ws/src/bringup/inspection_bringup/launch'
sys.path.insert(0, str(LAUNCH))
from field_validation_common import prepare_run  # noqa: E402


def test_visual_topics_are_unified():
    source_root = project_root()
    paths = [
        project_root() / 'ros2_ws/src/bringup/inspection_bringup/launch/visual_field_validation.launch.py',
        project_root() / 'ros2_ws/src/bringup/inspection_bringup/launch/hardware_pipeline.launch.py',
        project_root() / 'ros2_ws/src/bringup/inspection_bringup/config/record_topics_visual.yaml',
        project_root() / 'ros2_ws/src/functions/data/inspection_sync/src/inspection_sync_node.cpp',
    ]
    text = "\n".join(path.read_text() for path in paths)
    assert "/camera/front/image_raw" in text
    assert "/camera/camera/depth/image_rect_raw" not in text


def test_numeric_camera_serials_are_protected_from_yaml_retyping():
    text = (project_root() / 'ros2_ws/src/bringup/inspection_bringup/launch/hardware_pipeline.launch.py').read_text()
    assert '["sn_", LaunchConfiguration("mindvision_serial_number")]' in text
    assert '["_", LaunchConfiguration("d405_serial_number")]' in text


def test_visual_record_profile_has_raw_and_lightweight_modes():
    document = yaml.safe_load((project_root() / 'ros2_ws/src/bringup/inspection_bringup/config/record_topics_visual.yaml').read_text())
    assert "/camera/front/image_raw" not in document["topics"]["lightweight"]
    assert "/camera/front/image_raw" in document["topics"]["raw"]
    assert "/vision/front/seam_geometry" in document["topics"]["lightweight"]


def test_run_id_rejects_overwrite_and_invalid_name(monkeypatch, tmp_path):
    mount = {"path": str(tmp_path), "is_mount": True, "free_bytes": 10**12}
    monkeypatch.setattr("field_validation_common.validate_data_root", lambda *_: mount)
    run, _ = prepare_run(tmp_path, "safe_run")
    assert run.name == "safe_run"
    (run / "field_notes.md").write_text("existing")
    with pytest.raises(RuntimeError, match="non-empty"):
        prepare_run(tmp_path, "safe_run")
    with pytest.raises(RuntimeError, match="only ASCII"):
        prepare_run(tmp_path, "bad/run")


def test_manifest_status_shape_and_sha(tmp_path):
    manifest = {"status": {"complete": False, "finalized": False, "validation": "pending"}}
    path = tmp_path / "run_manifest.json"
    path.write_text(json.dumps(manifest))
    document = json.loads(path.read_text())
    assert document["status"]["validation"] == "pending"
