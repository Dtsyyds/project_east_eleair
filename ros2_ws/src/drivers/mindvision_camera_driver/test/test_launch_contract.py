from inspection_runtime.paths import project_root
from pathlib import Path


def test_all_digit_serial_is_forced_to_string_in_launch_override():
    launch = project_root() / 'ros2_ws/src/drivers/mindvision_camera_driver/launch/mindvision_camera.launch.py'
    text = launch.read_text(encoding="utf-8")
    assert 'f"sn_{value}" if name == "serial_number"' in text
