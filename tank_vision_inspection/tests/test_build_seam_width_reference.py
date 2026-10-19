from __future__ import annotations

import json
from pathlib import Path

from scripts.build_seam_width_reference import build
from tank_vision.seam_width_reference import sha256_file


def test_builder_uses_frozen_annotation_without_modifying_inputs(tmp_path: Path) -> None:
    root = Path(__file__).parents[1]
    v2 = root / "output/s1_d7_manual_ground_truth_v2"
    annotation = v2 / "seam_center_lit_quality_01_annotation_v2.json"
    manifest = v2 / "ground_truth_manifest.json"
    before = (sha256_file(annotation), sha256_file(manifest))
    output = tmp_path / "reference.json"
    result = build(annotation, manifest, output, sample_count=16)
    assert (sha256_file(annotation), sha256_file(manifest)) == before
    assert result["reference_is_manual"] is True
    assert all(result["frozen_v2_validation"].values())
    widths = [item["width_px_at_reference_resolution"] for item in result["reference_samples"]]
    assert len(widths) == 16 and min(widths) < max(widths)
    assert json.loads(output.read_text())["annotation_sha256"] == before[0]
