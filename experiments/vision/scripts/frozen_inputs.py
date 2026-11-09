"""Audit the single SDK-path change between frozen and migrated vision YAML."""
import copy
import hashlib
import os
from pathlib import Path

import yaml


def verify_config(path: Path, expected_sha256: str) -> dict:
    current_bytes = path.read_bytes()
    current_sha = hashlib.sha256(current_bytes).hexdigest()
    audit = {"current_path": str(path.resolve()), "current_sha256": current_sha,
             "frozen_sha256": expected_sha256, "changed_fields": []}
    if current_sha == expected_sha256:
        return audit
    fixture_root = os.environ.get("INSPECTION_BASELINE_FIXTURES")
    if not fixture_root:
        raise RuntimeError("Set INSPECTION_BASELINE_FIXTURES to verify the frozen config migration")
    frozen = Path(fixture_root) / "tank_vision_inspection/configs" / path.name
    frozen_bytes = frozen.read_bytes()
    if hashlib.sha256(frozen_bytes).hexdigest() != expected_sha256:
        raise RuntimeError("frozen config SHA mismatch: " + str(frozen))
    previous = yaml.safe_load(frozen_bytes)
    current = yaml.safe_load(current_bytes)
    expected = copy.deepcopy(previous)
    expected["camera"]["sdk_root"] = None
    if current != expected or current == previous:
        raise RuntimeError("config differs beyond the audited camera.sdk_root migration: " + str(path))
    audit.update(frozen_path=str(frozen.resolve()), changed_fields=["camera.sdk_root"],
                 sdk_root_before=previous["camera"]["sdk_root"], sdk_root_after=None)
    return audit
