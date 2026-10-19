from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo

from inspection_tools.stability_monitor import TopicState, parse_cdr_header
from inspection_tools.livox_continuity import classify_livox_continuity
from inspection_tools.time_readiness import evaluate_time_readiness
from inspection_tools.time_sync_analysis import nearest_differences
from inspection_tools.usb_lifecycle import analyze_logs, classify_log_line, enumerate_usb


def test_parse_serialized_header_without_payload_deserialization():
    message = CameraInfo()
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.header.frame_id = "camera_optical"
    stamp, frame = parse_cdr_header(serialize_message(message))
    assert stamp == 123_000_000_456
    assert frame == "camera_optical"


def test_nearest_timestamp_differences_are_signed_milliseconds():
    assert nearest_differences([10_000_000, 30_000_000], [9_000_000, 32_000_000]) == [1.0, -2.0]


def test_acceptance_rate_uses_complete_window_and_exposes_end_gap():
    state = TopicState({
        "topic": "/sensor", "message_type": "sensor_msgs/msg/CameraInfo",
        "source_device": "fixture", "expected_min_hz": 9.0,
    })
    # 10 Hz for one second, followed by a nine-second outage.
    for index in range(11):
        steady = 1_000_000_000 + index * 100_000_000
        state.update(steady, "frame", steady, steady, None)
    report = state.report(10.0, 11_000_000_000)
    assert report["active_span_rate_hz"] == 10.0
    assert report["actual_rate_hz"] == 1.1
    assert report["end_of_window_lag_sec"] == 9.0
    assert report["rate_pass"] is False


def test_livox_classifier_distinguishes_source_stop_from_monitor_stall():
    monitor = {
        "/livox/lidar": {"end_of_window_lag_sec": 500.0},
        "/livox/imu": {"end_of_window_lag_sec": 500.0},
    }
    stopped_probe = {
        "topics": {
            "/livox/lidar": {"end_of_window_lag_sec": 500.0},
            "/livox/imu": {"end_of_window_lag_sec": 500.0},
        },
        "diagnostics": {
            "livox_ros_driver2/publisher": {
                "message": "SOURCE_STOPPED", "values": {"classification": "SOURCE_STOPPED"}
            }
        },
    }
    assert classify_livox_continuity(monitor, stopped_probe)["classification"] == "SOURCE_STOPPED"

    streaming_probe = {
        "topics": {
            "/livox/lidar": {"end_of_window_lag_sec": 0.1},
            "/livox/imu": {"end_of_window_lag_sec": 0.1},
        },
        "diagnostics": {
            "livox_ros_driver2/publisher": {
                "message": "SOURCE_STREAMING", "values": {"classification": "SOURCE_STREAMING"}
            }
        },
    }
    classification = classify_livox_continuity(monitor, streaming_probe)["classification"]
    assert classification == "MONITOR_STALLED"


def test_time_readiness_requires_plausible_epoch_and_sync():
    ready = evaluate_time_readiness(
        system_time_ns=1_800_000_000_000_000_000, ntp_synchronized=True,
        rtc_time_ns=86_400_000_000_000)
    assert ready["status"] == "TIME_READY"
    assert ready["rtc_valid"] is False
    assert evaluate_time_readiness(
        system_time_ns=86_400_000_000_000, ntp_synchronized=False)["status"] == "TIME_NOT_READY"
    not_synchronized = evaluate_time_readiness(
        system_time_ns=1_800_000_000_000_000_000, ntp_synchronized=False)
    assert not_synchronized["status"] == "TIME_NOT_READY"


def test_usb_enumeration_reads_target_and_video_nodes(tmp_path):
    device = tmp_path / "4-1"
    video = device / "4-1:1.0" / "video4linux" / "video0"
    video.mkdir(parents=True)
    (device / "idVendor").write_text("8086\n")
    (device / "idProduct").write_text("0b5b\n")
    (device / "speed").write_text("5000\n")
    (device / "serial").write_text("260322276866\n")
    devices = enumerate_usb("8086", "0b5b", tmp_path)
    assert len(devices) == 1
    assert devices[0]["speed"] == "5000"
    assert devices[0]["video_nodes"] == ["/dev/video0"]


def test_lifecycle_log_classification_and_chronological_sort(tmp_path):
    log = tmp_path / "camera.log"
    log.write_text(
        "[INFO] [1791886677.0] RealSense Node Is Up!\n"
        "[ERROR] [1791886683.0] get_xu(...) UVCIOC_CTRL_QUERY Connection timed out\n"
    )
    assert "UVC_CONTROL_ERROR" in classify_log_line(log.read_text().splitlines()[1])
    report = analyze_logs([log])
    assert [event["event"] for event in report["events"]] == [
        "PIPELINE_UP", "UVC_CONTROL_ERROR", "USB_TIMEOUT"]
    assert report["events"][0]["system_timestamp_ns"] == 1791886677000000000
