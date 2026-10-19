#!/usr/bin/env python3
"""Validate a bounded live MindVision JPEG stream from tank_inspection_v2."""

import argparse
import base64
import json
import time

import cv2
import numpy as np
from websockets.sync.client import connect


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--uri", default="ws://127.0.0.1:9001")
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    args = parser.parse_args()
    request_id = "mindvision-live-validation"
    frames = []
    status = None
    first_shape = None
    first_mean_bgr = None
    started = time.monotonic()
    with connect(args.uri, proxy=None, open_timeout=3) as websocket:
        websocket.send(json.dumps({
            "header": {
                "robot_id": "tank_inspection_robot", "msg_type": "check",
                "schema_version": "tank_inspection_v2", "request_id": request_id,
            },
            "payload": {},
        }))
        deadline = started + args.timeout_s
        while time.monotonic() < deadline and (status is None or len(frames) < args.frames):
            try:
                packet = json.loads(websocket.recv(timeout=0.5))
            except TimeoutError:
                continue
            header = packet.get("header", {})
            payload = packet.get("payload", {})
            if header.get("request_id") == request_id and header.get("msg_type") == "response":
                status = payload.get("mindvision")
            video = payload.get("video")
            if not video or video.get("camera_id") != "mindvision":
                continue
            decoded = cv2.imdecode(
                np.frombuffer(base64.b64decode(video["data"]), dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if decoded is None:
                raise RuntimeError("MindVision WebSocket JPEG could not be decoded")
            if first_shape is None:
                first_shape = list(decoded.shape)
                first_mean_bgr = decoded.mean(axis=(0, 1)).round(3).tolist()
            frames.append({
                "received_at": time.monotonic(),
                "stamp_ns": video["stamp_ns"],
                "width": video["width"], "height": video["height"],
                "source_received": video["source_received"],
                "source_accepted": video["source_accepted"],
                "preview_dropped": video["preview_dropped"],
                "schema_version": header.get("schema_version"),
            })
    elapsed = frames[-1]["received_at"] - frames[0]["received_at"] if len(frames) > 1 else 0.0
    fps = (len(frames) - 1) / elapsed if elapsed > 0 else 0.0
    result = {
        "status": status, "frames": len(frames), "receive_fps": round(fps, 3),
        "first_shape": first_shape, "first_mean_bgr": first_mean_bgr,
        "first_frame": frames[0] if frames else None,
        "last_frame": frames[-1] if frames else None,
    }
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    valid = (
        status is not None and status.get("status") == "verified"
        and len(frames) >= args.frames and first_shape is not None
        and all(frame["schema_version"] == "tank_inspection_v2" for frame in frames)
    )
    return 0 if valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
