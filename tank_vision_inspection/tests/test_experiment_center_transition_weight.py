"""专项测试：S1-P2E2 只覆盖 center transition weight 的内存消融。"""

from __future__ import annotations

import copy
import hashlib
import json
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.experiment_center_transition_weight import (  # noqa: E402
    GUIDED_CONFIG,
    OUT,
    WEIGHTS,
    config_for,
)


def test_only_center_transition_weight_is_overridden() -> None:
    original = yaml.safe_load(GUIDED_CONFIG.read_text(encoding="utf-8"))
    for weight in WEIGHTS:
        modified = config_for(weight)
        expected = copy.deepcopy(original)
        expected["path"]["center_transition_weight"] = weight
        assert modified == expected


def test_experiment_uses_exact_five_weights() -> None:
    assert WEIGHTS == (0.42, 0.32, 0.24, 0.16, 0.08)


def test_completed_report_has_all_weights_and_no_yaml_change() -> None:
    summary = json.loads((OUT / "diagnostic_summary.json").read_text(encoding="utf-8"))
    comparison = json.loads((OUT / "weight_comparison.json").read_text(encoding="utf-8"))
    assert summary["baseline_gate"]["status"] == "matched"
    assert [row["weight"] for row in comparison["rows"]] == list(WEIGHTS)
    assert summary["production_changes"]["yaml"] is False


def test_fixed_input_hashes_match_before_after_artifact() -> None:
    payload = json.loads((OUT / "input_sha256_after.json").read_text(encoding="utf-8"))
    assert payload["unchanged"] is True
    assert payload["before"] == payload["after"]
