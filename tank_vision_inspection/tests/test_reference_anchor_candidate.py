"""专项测试：S1-P2E4 reference-anchored candidate branch."""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.experiment_reference_anchor_candidate import CONFIG, OUT, RAW, REFERENCE, config_for  # noqa: E402
from tank_vision.stable_seam_tracker import (  # noqa: E402
    StableSeamTracker, _find_reference_anchor, _merge_reference_anchor,
)


def test_anchor_disabled_is_fieldwise_identical_to_baseline() -> None:
    image = cv2.imread(str(RAW), cv2.IMREAD_COLOR)
    assert image is not None
    plain = StableSeamTracker(CONFIG, REFERENCE)
    explicit = StableSeamTracker(config_for(False), REFERENCE)
    plain_result = plain.process(image, diagnostics=True, provenance=True)
    explicit_result = explicit.process(image, diagnostics=True, provenance=True)
    plain_fields = plain_result.to_dict()
    explicit_fields = explicit_result.to_dict()
    for fields in (plain_fields, explicit_fields):
        fields.pop("processing_ms", None)
        fields.pop("stage_timings_ms", None)
    assert plain_fields == explicit_fields
    assert plain.last_diagnostics is not None and explicit.last_diagnostics is not None
    assert [[(p.left, p.right, p.score, p.score_parts) for p in row] for row in plain.last_diagnostics.all_pairs] == [[(p.left, p.right, p.score, p.score_parts) for p in row] for row in explicit.last_diagnostics.all_pairs]
    assert plain.last_provenance == explicit.last_provenance


def test_anchor_requires_direction_and_minimum_response_and_margin() -> None:
    profile = np.zeros(64, dtype=np.float32)
    profile[20] = 2.0
    profile[40] = -3.0
    assert _find_reference_anchor(profile, 10, 30, True, 1.0, 2)["position_processed"] == 20
    assert _find_reference_anchor(profile, 30, 50, False, 1.0, 2)["position_processed"] == 40
    assert not _find_reference_anchor(profile, 10, 30, False, 1.0, 2)["valid"]
    assert not _find_reference_anchor(profile, 0, 5, True, 1.0, 5)["valid"]
    assert not _find_reference_anchor(np.zeros(64, dtype=np.float32), 10, 30, True, 1.0, 2)["valid"]


def test_anchor_merge_deduplicates_and_keeps_three_candidates() -> None:
    profile = np.zeros(64, dtype=np.float32)
    profile[[10, 20, 30, 40]] = [1, 2, 3, 4]
    merged, detail = _merge_reference_anchor(np.asarray([10, 20, 30], dtype=np.int32),
                                             {"position_processed": 21, "signed_response": 2, "strength": 2, "valid": True},
                                             profile, True, 5, 3)
    assert len(merged) <= 3
    assert detail["merged_with_existing"] is True
    fresh, detail = _merge_reference_anchor(np.asarray([10, 20, 30], dtype=np.int32),
                                            {"position_processed": 40, "signed_response": 4, "strength": 4, "valid": True},
                                            profile, True, 2, 3)
    assert len(fresh) == 3 and 40 in fresh
    assert detail["retained_in_final_candidates"] is True


def test_completed_ab_has_required_outputs_and_hard_negative_passes() -> None:
    summary = json.loads((OUT / "diagnostic_summary.json").read_text(encoding="utf-8"))
    assert summary["baseline_status"] == "matched"
    assert summary["fixed_input_hashes_unchanged"] is True
    assert summary["production_changes"] == {"yaml": False, "reference": False, "dp": False, "unary": False, "gate": False, "ros": False}
    negative = json.loads((OUT / "negative_regression.json").read_text(encoding="utf-8"))
    assert negative["A"]["hard_negative_pass"] and negative["B"]["hard_negative_pass"]
    assert (OUT / "anchor_candidate_trace.json").is_file()
    assert (OUT / "key_band_profile_montage.png").is_file()


def test_experiment_config_only_adds_anchor_option_in_memory() -> None:
    original = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    disabled = config_for(False)
    enabled = config_for(True)
    assert disabled == original
    expected = copy.deepcopy(original)
    expected["guided_search"]["reference_anchor_enabled"] = True
    expected["guided_search"]["reference_anchor_radius_width_scale"] = 0.12
    assert enabled == expected
