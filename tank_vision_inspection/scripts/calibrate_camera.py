#!/usr/bin/env python3
"""Calibrate a pinhole camera from chessboard images."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
import numpy as np

from tank_vision.calibration import CameraCalibration, save_camera_calibration


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", help="Folder containing calibration images")
    parser.add_argument("output", help="Output YAML path")
    parser.add_argument("--columns", type=int, required=True, help="Inner chessboard corners per row")
    parser.add_argument("--rows", type=int, required=True, help="Inner chessboard corners per column")
    parser.add_argument("--square-size", type=float, required=True, help="Square size in a user-selected unit")
    args = parser.parse_args()
    if args.columns < 2 or args.rows < 2 or args.square_size <= 0:
        parser.error("board dimensions and square size must be positive")
    paths = sorted(path for path in Path(args.images).glob("*") if path.is_file())
    object_template = np.zeros((args.rows * args.columns, 3), np.float32)
    object_template[:, :2] = np.mgrid[0 : args.columns, 0 : args.rows].T.reshape(-1, 2) * args.square_size
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        image_size = (image.shape[1], image.shape[0])
        found, corners = cv2.findChessboardCorners(image, (args.columns, args.rows))
        if not found:
            continue
        refined = cv2.cornerSubPix(
            image,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        object_points.append(object_template.copy())
        image_points.append(refined)
    if image_size is None or len(object_points) < 5:
        raise SystemExit(f"Need at least 5 valid chessboard images; found {len(object_points)}")
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(object_points, image_points, image_size, None, None)
    calibration = CameraCalibration(
        camera_matrix=matrix,
        distortion_coefficients=distortion,
        image_width=image_size[0],
        image_height=image_size[1],
        version=datetime.now(timezone.utc).strftime("calib-%Y%m%dT%H%M%SZ"),
        rms_error=float(rms),
    )
    save_camera_calibration(calibration, args.output)
    print(f"Saved calibration from {len(object_points)} images; RMS={rms:.4f} px")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
