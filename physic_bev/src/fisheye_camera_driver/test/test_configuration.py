from pathlib import Path

import pytest

from fisheye_camera_driver.configuration import load_camera_info, resolve_device, validate_capture


def test_stable_device_resolution(tmp_path):
    target = tmp_path / "usb-icSpring_camera_SERIAL-video-index0"
    target.symlink_to("../../video6")
    assert resolve_device("", "SERIAL", tmp_path) == target
    assert resolve_device("/dev/video9", "ignored", tmp_path) == Path("/dev/video9")


def test_stable_device_resolution_rejects_ambiguous_or_missing(tmp_path):
    with pytest.raises(RuntimeError, match="matched 0"):
        resolve_device("", "fisheye", tmp_path)
    for number in (1, 2):
        (tmp_path / f"usb-fisheye-{number}-video-index0").symlink_to(f"../../video{number}")
    with pytest.raises(RuntimeError, match="matched 2"):
        resolve_device("", "fisheye", tmp_path)


def test_capture_contract_validation():
    assert validate_capture(1920, 1080, 15.0, "mjpg") == (1920, 1080, 15.0, "MJPG")
    with pytest.raises(ValueError):
        validate_capture(0, 1080, 15.0, "MJPG")
    with pytest.raises(ValueError):
        validate_capture(1920, 1080, 15.0, "H264")


def test_calibration_is_optional_and_strict(tmp_path):
    assert load_camera_info("") is None
    calibration = tmp_path / "fisheye.yaml"
    calibration.write_text("""
image_width: 640
image_height: 480
distortion_model: equidistant
camera_matrix: {data: [1, 0, 2, 0, 1, 3, 0, 0, 1]}
distortion_coefficients: {data: [0, 0, 0, 0]}
rectification_matrix: {data: [1, 0, 0, 0, 1, 0, 0, 0, 1]}
projection_matrix: {data: [1, 0, 2, 0, 0, 1, 3, 0, 0, 0, 1, 0]}
""")
    assert load_camera_info(str(calibration))["distortion_model"] == "equidistant"
    calibration.write_text("image_width: 640\n")
    with pytest.raises(ValueError, match="missing fields"):
        load_camera_info(str(calibration))
