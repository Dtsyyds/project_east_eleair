"""Tests for the independent S1-P2F0 ROS validation candidate."""

from __future__ import annotations

import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "output/s1_p2f0_guided_anchor_ros_candidate"
GUIDED = ROOT / "configs/s100p_guided_seam_4096x3072.yaml"
ANCHOR = ROOT / "configs/s100p_guided_anchor_seam_4096x3072.yaml"
STABLE = ROOT / "configs/s100p_stable_seam_4096x3072.yaml"
LAUNCH = ROOT / "ros2_ws/src/tank_vision_ros/launch/stable_seam_direct_guided_anchor.launch.py"
ORIGINAL_LAUNCH = ROOT / "ros2_ws/src/tank_vision_ros/launch/stable_seam_direct.launch.py"
SETUP = ROOT / "ros2_ws/src/tank_vision_ros/setup.py"


def load(name: str):
    return json.loads((OUT / name).read_text(encoding="utf-8"))


def test_candidate_yaml_is_guided_plus_only_anchor_fields() -> None:
    guided = yaml.safe_load(GUIDED.read_text(encoding="utf-8"))
    candidate = yaml.safe_load(ANCHOR.read_text(encoding="utf-8"))
    expected = dict(guided)
    expected["guided_search"] = dict(guided["guided_search"])
    expected["guided_search"].update({"reference_anchor_enabled": True, "reference_anchor_radius_width_scale": 0.12})
    assert candidate == expected
    assert candidate["reference"]["path"].endswith("seam_geometry_reference_s100p_live_v1.json")
    assert candidate["path"]["center_transition_weight"] == 0.42
    assert "experimental field-validation candidate, not production default" in ANCHOR.read_text(encoding="utf-8").splitlines()[0]
    assert yaml.safe_load(STABLE.read_text(encoding="utf-8")).get("guided_search", {}).get("reference_anchor_enabled", False) is False


def test_independent_launch_arguments_and_install_rules() -> None:
    source = LAUNCH.read_text(encoding="utf-8")
    for argument in ("config_file", "publish_debug_image", "snapshot_output_root"):
        assert f'DeclareLaunchArgument("{argument}"' in source
    assert "s100p_guided_anchor_seam_4096x3072.yaml" in source
    assert "seam_geometry_reference_s100p_live_v1.json" in source
    assert "input_mode\": \"mindvision_direct\"" in source
    setup = SETUP.read_text(encoding="utf-8")
    assert "stable_seam_direct_guided_anchor.launch.py" in setup
    assert "s100p_guided_anchor_seam_4096x3072.yaml" in setup
    assert "seam_geometry_reference_s100p_live_v1.json" in setup
    assert "stable_seam_direct_guided_anchor.launch.py" not in ORIGINAL_LAUNCH.read_text(encoding="utf-8")


def test_validation_outputs_pass_and_default_path_is_separate() -> None:
    summary = load("validation_summary.json")
    assert summary["status"] == "passed"
    assert summary["candidate_status"] == "field-validation candidate"
    assert summary["camera_started"] is False
    offline = load("offline_reproduction.json")
    assert offline["status"] == "matched"
    assert offline["hard_no_seam_negative"]["pass"] is True
    runtime = load("runtime_config_resolution.json")
    assert runtime["show_args_returncode"] == 0
    assert runtime["show_args_has_required"] is True
    hashes = load("source_install_hashes.json")
    assert all(item["match"] for item in hashes.values())
    assert load("default_stable_regression.json")["anchor_enabled"] is False
    ros = load("ros_e2e_result.json")
    assert ros["status"] == "passed"
    assert ros["camera_started"] is False
