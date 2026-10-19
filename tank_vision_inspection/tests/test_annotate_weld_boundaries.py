from pathlib import Path

import numpy as np
import pytest

from scripts.annotate_weld_boundaries import (
    AnnotationState,
    display_to_original,
    original_to_display,
    validate_annotation,
)


def test_coordinate_scaling_round_trip() -> None:
    scale = 896 / 4096
    x, y = display_to_original(*original_to_display(1234.0, 987.0, scale), scale)
    assert abs(x - 1234.0) < 4.0
    assert abs(y - 987.0) < 4.0


def test_undo_only_removes_current_boundary() -> None:
    state = AnnotationState()
    state.add((10.0, 10.0))
    state.active = "right"
    state.add((20.0, 10.0))
    state.undo()
    assert state.left == [(10.0, 10.0)]
    assert state.right == []


def test_json_validation_sorts_and_rejects_invalid_points() -> None:
    left = [{"x": float(100 - i), "y": float(20 * i)} for i in range(6)]
    right = [{"x": float(200 + i), "y": float(20 * i)} for i in range(6)]
    sorted_left, sorted_right = validate_annotation(left, right, 300, 200)
    assert [p["y"] for p in sorted_left] == sorted(p["y"] for p in sorted_left)
    assert [p["y"] for p in sorted_right] == sorted(p["y"] for p in sorted_right)
    with pytest.raises(ValueError, match="at least 6"):
        validate_annotation(left[:5], right, 300, 200)
    with pytest.raises(ValueError, match="out-of-bounds"):
        validate_annotation(left, right[:-1] + [{"x": 301, "y": 100}], 300, 200)
