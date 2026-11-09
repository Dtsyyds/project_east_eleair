#!/usr/bin/env python3
"""Extract video frames and a media-timeline timestamp manifest."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("video")
    parser.add_argument("output")
    parser.add_argument("--every", type=int, default=1, help="Save every Nth frame")
    args = parser.parse_args()
    if args.every <= 0:
        parser.error("--every must be positive")
    video_path = Path(args.video)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise SystemExit(f"Could not open video: {video_path}")
    saved = 0
    index = 0
    with (output / "timestamps.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["filename", "source_frame_index", "media_timestamp_ns"])
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            timestamp_ns = int(max(0.0, capture.get(cv2.CAP_PROP_POS_MSEC)) * 1_000_000)
            if index % args.every == 0:
                name = f"frame_{index:08d}.png"
                if not cv2.imwrite(str(output / name), frame):
                    raise OSError(f"Could not write {name}")
                writer.writerow([name, index, timestamp_ns])
                saved += 1
            index += 1
    capture.release()
    print(f"Extracted {saved} frames to {output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

