import json
from pathlib import Path
import struct
import sys
import threading
import time

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from replay import manifest


def test_manifest_rejects_traversal_and_active_recording(tmp_path):
    for name in ("../../other", "WS-../../other", "/tmp/other"):
        with pytest.raises(ValueError):
            manifest(tmp_path, name)
    run = tmp_path / "runs/WS-test"
    run.mkdir(parents=True)
    (run / "run.json").write_text(json.dumps({"status": "recording"}))
    with pytest.raises(ValueError, match="尚未停止"):
        manifest(tmp_path, "WS-test")


def test_manifest_rejects_file_escape_and_unclosed_file(tmp_path):
    run = tmp_path / "runs/WS-test"
    bag = run / "bag"
    bag.mkdir(parents=True)
    (run / "run.json").write_text(json.dumps({"status": "complete"}))
    meta = {"storage_identifier": "mcap", "relative_file_paths": ["../../escape.mcap"]}
    (bag / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": meta}))
    with pytest.raises(ValueError, match="路径越界"):
        manifest(tmp_path, "WS-test")
    meta["relative_file_paths"] = ["partial.mcap"]
    (bag / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": meta}))
    (bag / "partial.mcap").write_bytes(b"\x89MCAP0\r\n" + b"unfinished")
    with pytest.raises(ValueError, match="尚未正常关闭"):
        manifest(tmp_path, "WS-test")


def test_sensor_codecs_handle_endian_padding_and_interleaved_channels():
    pytest.importorskip("rosbag2_py")
    import numpy as np
    import cv2
    from sensor_msgs.msg import Image, PointCloud2, PointField
    from eddy_driver.msg import EddyFrame
    from inspection_interfaces.msg import UltrasoundFrame
    from replay_worker import image_preview, pointcloud_preview, sensor_preview
    depth = np.array([[1000, 1300, 0], [2000, 3000, 0]], dtype=">u2")
    padded = b"".join(row.tobytes() + b"\0\0" for row in depth)
    image = Image(width=3, height=2, encoding="16UC1", is_bigendian=1, step=8, data=padded)
    preview = image_preview(image)
    decoded = cv2.imdecode(np.frombuffer(__import__("base64").b64decode(preview["jpeg"]), np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (2, 3, 3)
    cloud = PointCloud2(height=2, width=2, is_bigendian=True, point_step=16, row_step=36,
                       fields=[PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(("x", "y", "z"))],
                       data=b"".join(struct.pack(">fffIfffI", 1., 2., 3., 0, 4., 5., 6., 0) + b"\0"*4 for _ in range(2)))
    assert pointcloud_preview(cloud)["xyz"] == [1., 2., 3., 4., 5., 6.] * 2
    eddy = EddyFrame(channel_count=8, samples_per_channel=20, data=list(range(160)))
    channels = sensor_preview("eddy_raw", eddy)["channels"]
    assert channels[0] == list(range(0, 160, 8))
    assert channels[7] == list(range(7, 160, 8))
    emat = UltrasoundFrame(channel_count=2, sample_count=3, samples=[1, 2, 3, 10, 20, 30])
    assert sensor_preview("emat_raw", emat)["channels"] == [[1, 2, 3], [10, 20, 30]]


def test_paused_clock_waits_and_stop_wakes_reader():
    pytest.importorskip("rosbag2_py")
    from replay_worker import Playback
    packets = []
    player = Playback("/not-opened", "test", 1, packets.append)
    player.duration = 10
    player.wall_origin = time.monotonic()
    player.command("pause")
    returned = threading.Event()
    thread = threading.Thread(target=lambda: (player.wait(0), returned.set()))
    thread.start()
    assert not returned.wait(.1)
    player.command("stop")
    assert returned.wait(1)
    thread.join()
    assert packets[0]["data"]["status"] == "paused"
