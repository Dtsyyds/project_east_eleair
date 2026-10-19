from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from tank_vision.config import load_config


def test_default_config_loads_and_validates() -> None:
    config = load_config(Path(__file__).parents[1] / "configs" / "default.yaml")
    assert config.camera.width == 4096
    assert config.defect.tile_size == 1024
    assert config.pipeline.roi is None


def test_s100p_realtime_profile_only_changes_requested_runtime_values() -> None:
    root = Path(__file__).parents[1]
    default = yaml.safe_load((root / "configs" / "default.yaml").read_text(encoding="utf-8"))
    profile = yaml.safe_load((root / "configs" / "s100p_realtime_1920x1200.yaml").read_text(encoding="utf-8"))
    allowed_changes = {
        ("camera", "width"),
        ("camera", "height"),
        ("inspection", "enabled"),
        ("pipeline", "roi"),
        ("pipeline", "thumbnail_max_dimension"),
        ("pipeline", "save_overlay"),
        ("pipeline", "save_json"),
        ("pipeline", "save_masks"),
    }
    assert profile["camera"]["width"] == 1920
    assert profile["camera"]["height"] == 1200
    assert profile["inspection"]["enabled"] is False
    assert profile["pipeline"]["roi"] is None
    assert profile["pipeline"]["thumbnail_max_dimension"] == 1024
    assert profile["pipeline"]["save_overlay"] is False
    assert profile["pipeline"]["save_json"] is False
    assert profile["pipeline"]["save_masks"] is False
    for section_name, section in default.items():
        for key, value in section.items():
            if (section_name, key) not in allowed_changes:
                assert profile[section_name][key] == value, f"unexpected profile change: {section_name}.{key}"
    assert load_config(root / "configs" / "s100p_realtime_1920x1200.yaml").inspection.enabled is False


def test_invalid_overlap_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump({"defect": {"tile_size": 128, "overlap": 128}}), encoding="utf-8")
    with pytest.raises(ValueError, match="overlap"):
        load_config(path)


def test_unknown_keys_are_rejected(tmp_path: Path) -> None:
    path = tmp_path / "unknown.yaml"
    path.write_text("seam:\n  imaginary_threshold: 3\n", encoding="utf-8")
    with pytest.raises(ValueError, match="Unknown keys"):
        load_config(path)
