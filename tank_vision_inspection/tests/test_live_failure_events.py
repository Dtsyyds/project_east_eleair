from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "ros2_ws/src/tank_vision_ros/tank_vision_ros/stable_seam_node.py"
LAUNCH = ROOT / "ros2_ws/src/tank_vision_ros/launch/stable_seam_direct_guided_anchor.launch.py"


def test_live_session_parameters_and_bounded_async_writers_are_declared() -> None:
    source = NODE.read_text(encoding="utf-8")
    for parameter in ("auto_save_failures", "failure_snapshot_cooldown_sec", "max_failure_snapshots_per_session",
                      "auto_save_full_frame", "auto_save_context_jpeg", "save_lossless_roi",
                      "live_session_output_root", "per_frame_metrics_log"):
        assert f'declare_parameter("{parameter}"' in source or f'declare_parameter(\n            "{parameter}"' in source
    assert "LiveSessionRecorder" in source
    assert "diagnostics=False, provenance=False" in source
    assert "Thread(target=self._direct_capture_worker" in source
    assert "Queue(maxsize=4)" in source


def test_direct_capture_does_not_run_tracker_inline() -> None:
    source = NODE.read_text(encoding="utf-8")
    direct = source.split("def _direct_capture_worker", 1)[1].split("def destroy_node", 1)[0]
    assert "self._enqueue_packet" in direct
    assert "self.tracker.process" not in direct


def test_guided_launch_exposes_live_failure_controls_without_changing_stable_launch() -> None:
    source = LAUNCH.read_text(encoding="utf-8")
    for parameter in ("publish_debug_image", "auto_save_failures", "failure_snapshot_cooldown_sec",
                      "max_failure_snapshots_per_session", "auto_save_full_frame",
                      "auto_save_context_jpeg", "save_lossless_roi", "auto_save_policy", "live_session_output_root"):
        assert f'DeclareLaunchArgument("{parameter}"' in source
