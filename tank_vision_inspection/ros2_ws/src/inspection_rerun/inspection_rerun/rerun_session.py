"""Rerun 0.36 session ownership isolated from ROS and acquisition nodes."""

from pathlib import Path
import threading

import numpy as np
import rerun as rr


class RerunSession:
    def __init__(self, output_mode, recording_id, save_path, connect_url="rerun+http://127.0.0.1:9876/proxy"):
        if output_mode not in {"spawn", "connect", "save"}:
            raise ValueError("output_mode must be spawn, connect or save")
        self.output_mode = output_mode
        self._lock = threading.RLock()
        self.closed = False
        if output_mode == "save":
            # Use the documented headless path explicitly.  This never starts
            # the native Viewer and gives the offline builder a footer-bearing
            # file after flush/disconnect.
            rr.init(
                "tank_vision_inspection", recording_id=recording_id,
                spawn=False, default_enabled=True,
            )
            self.recording = rr.get_global_data_recording()
            if self.recording is None:
                raise RuntimeError("Rerun did not create a global recording stream")
        else:
            self.recording = rr.RecordingStream(
                "tank_vision_inspection", recording_id=recording_id)
        if output_mode == "spawn":
            self.recording.spawn()
        elif output_mode == "connect":
            self.recording.connect_grpc(connect_url)
        else:
            path = Path(save_path).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.save_path = path
            rr.save(path, recording=self.recording, write_footer=True)

    def set_time(self, timestamp_ns, sequence):
        self.recording.set_time("ros_time", timestamp=np.datetime64(int(timestamp_ns), "ns"))
        self.recording.set_time("frame_sequence", sequence=int(sequence))

    def log(self, path, archetype, *, static=False):
        with self._lock:
            if not self.closed:
                self.recording.log(path, archetype, static=static)

    def scalar(self, path, value):
        self.log(path, rr.Scalars([float(value)]))

    def text(self, path, text, level="INFO"):
        self.log(path, rr.TextLog(str(text), level=level))

    def close(self):
        with self._lock:
            if self.closed:
                return
            self.closed = True
            self.recording.flush(timeout_sec=10.0)
            self.recording.disconnect()
