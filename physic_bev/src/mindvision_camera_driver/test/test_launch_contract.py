from pathlib import Path


def test_all_digit_serial_is_forced_to_string_in_launch_override():
    launch = Path(__file__).parents[1] / "launch" / "mindvision_camera.launch.py"
    text = launch.read_text(encoding="utf-8")
    assert 'f"sn_{value}" if name == "serial_number"' in text
