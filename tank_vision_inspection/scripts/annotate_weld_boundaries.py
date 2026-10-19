#!/usr/bin/env python3
"""Small interactive, original-coordinate weld-boundary annotation tool.

This is a manual ground-truth utility only.  It does not call any detector.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = ROOT / "output/s1_real_seam_images/seam_center_lit_quality_01.bmp"
DEFAULT_OUTPUT = ROOT / "output/s1_d7_manual_ground_truth"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def display_scale(width: int, height: int, max_width: int = 1400, max_height: int = 900) -> float:
    """Return an aspect-preserving display scale, never enlarging the image."""
    return min(1.0, max_width / width, max_height / height)


def display_to_original(x: float, y: float, scale: float) -> tuple[float, float]:
    if scale <= 0:
        raise ValueError("display scale must be positive")
    return (float(x / scale), float(y / scale))


def original_to_display(x: float, y: float, scale: float) -> tuple[int, int]:
    if scale <= 0:
        raise ValueError("display scale must be positive")
    return (int(round(x * scale)), int(round(y * scale)))


def _point_dict(point: Iterable[float]) -> dict[str, float]:
    x, y = point
    return {"x": float(x), "y": float(y)}


def validate_annotation(left: list[dict], right: list[dict], width: int, height: int) -> tuple[list[dict], list[dict]]:
    if len(left) < 6 or len(right) < 6:
        raise ValueError("both boundaries require at least 6 manual points")

    def normalize(points: list[dict], name: str) -> list[dict]:
        output = []
        for point in points:
            x, y = float(point["x"]), float(point["y"])
            if not (np.isfinite(x) and np.isfinite(y)):
                raise ValueError(f"{name} contains a non-finite coordinate")
            if not (0 <= x < width and 0 <= y < height):
                raise ValueError(f"{name} contains an out-of-bounds coordinate")
            output.append(_point_dict((x, y)))
        return sorted(output, key=lambda item: (item["y"], item["x"]))

    return normalize(left, "left_boundary_manual_points"), normalize(right, "right_boundary_manual_points")


@dataclass
class AnnotationState:
    left: list[tuple[float, float]] = field(default_factory=list)
    right: list[tuple[float, float]] = field(default_factory=list)
    active: str = "left"

    def add(self, point: tuple[float, float]) -> None:
        getattr(self, self.active).append(point)

    def undo(self) -> None:
        points = getattr(self, self.active)
        if points:
            points.pop()

    def clear(self) -> None:
        getattr(self, self.active).clear()


def _draw_polyline(canvas: np.ndarray, points: list[tuple[float, float]], scale: float, colour: tuple[int, int, int]) -> None:
    if len(points) < 2:
        return
    display = np.asarray([original_to_display(x, y, scale) for x, y in points], dtype=np.int32)
    cv2.polylines(canvas, [display.reshape(-1, 1, 2)], False, colour, 2, cv2.LINE_AA)
    for x, y in display:
        cv2.circle(canvas, (int(x), int(y)), 4, colour, -1, cv2.LINE_AA)


def _draw_midpoint_reference(canvas: np.ndarray, state: AnnotationState, scale: float) -> None:
    # This is display-only guidance.  It does not create or save annotation points.
    if len(state.left) < 2 or len(state.right) < 2:
        return
    left = sorted(state.left, key=lambda p: p[1])
    right = sorted(state.right, key=lambda p: p[1])
    ys = sorted(set([p[1] for p in left] + [p[1] for p in right]))
    lx = np.interp(ys, [p[1] for p in left], [p[0] for p in left])
    rx = np.interp(ys, [p[1] for p in right], [p[0] for p in right])
    mid = [original_to_display((a + b) * 0.5, y, scale) for a, b, y in zip(lx, rx, ys) if a < b]
    if len(mid) >= 2:
        cv2.polylines(canvas, [np.asarray(mid, dtype=np.int32).reshape(-1, 1, 2)], False, (0, 255, 255), 1, cv2.LINE_AA)


def save_annotation(input_path: Path, output_dir: Path, image: np.ndarray, state: AnnotationState,
                    coordinate_system: str = "original_image_pixels") -> Path:
    height, width = image.shape[:2]
    left, right = validate_annotation(
        [_point_dict(p) for p in state.left], [_point_dict(p) for p in state.right], width, height
    )
    input_hash = sha256(input_path)
    payload = {
        "input_path": str(input_path),
        "input_sha256": input_hash,
        "image_width": width,
        "image_height": height,
        "coordinate_system": coordinate_system,
        "left_boundary_manual_points": left,
        "right_boundary_manual_points": right,
        "point_count": {"left": len(left), "right": len(right), "total": len(left) + len(right)},
        "created_at": datetime.now(timezone.utc).isoformat(),
        "annotation_type": "manual",
        "interpolation_used": False,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / f"{input_path.stem}_annotation.json"
    overlay = image.copy()
    scale = display_scale(width, height, width, height)
    _draw_polyline(overlay, [(p["x"], p["y"]) for p in left], scale, (255, 0, 0))
    _draw_polyline(overlay, [(p["x"], p["y"]) for p in right], scale, (0, 255, 0))
    _draw_midpoint_reference(overlay, state, scale)
    if not cv2.imwrite(str(output_dir / f"{input_path.stem}_annotation_overlay.png"), overlay):
        raise OSError("failed to write annotation overlay")
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return json_path


def annotate(input_path: Path, output_dir: Path, coordinate_system: str = "original_image_pixels") -> int:
    image = cv2.imread(str(input_path), cv2.IMREAD_COLOR)
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise RuntimeError(f"cannot read BGR image: {input_path}")
    height, width = image.shape[:2]
    scale = display_scale(width, height)
    display_size = (max(1, round(width * scale)), max(1, round(height * scale)))
    state = AnnotationState()
    window = "S1-D7 manual weld boundaries"

    def render() -> None:
        canvas = cv2.resize(image, display_size, interpolation=cv2.INTER_AREA)
        _draw_polyline(canvas, state.left, scale, (255, 0, 0))
        _draw_polyline(canvas, state.right, scale, (0, 255, 0))
        _draw_midpoint_reference(canvas, state, scale)
        lines = ["1=left  2=right  LMB=add  u=undo  r=clear  s=save  q/ESC=quit", f"active={state.active} | left={len(state.left)} right={len(state.right)}"]
        for index, text in enumerate(lines):
            cv2.putText(canvas, text, (12, 28 + index * 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(canvas, text, (12, 28 + index * 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.imshow(window, canvas)

    def mouse(event: int, x: int, y: int, _flags: int, _userdata: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            state.add(display_to_original(x, y, scale))
            render()

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, mouse)
    render()
    while True:
        key = cv2.waitKey(30) & 0xFF
        if key == ord("1"):
            state.active = "left"
        elif key == ord("2"):
            state.active = "right"
        elif key == ord("u"):
            state.undo()
        elif key == ord("r"):
            state.clear()
        elif key == ord("s"):
            try:
                save_annotation(input_path, output_dir, image, state, coordinate_system)
                print(f"saved annotation to {output_dir}")
            except ValueError as exc:
                print(f"not saved: {exc}")
        elif key in (ord("q"), 27):
            break
        render()
    cv2.destroyAllWindows()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--coordinate-system", choices=("original_image_pixels", "roi_pixels"),
                        default="original_image_pixels")
    args = parser.parse_args()
    return annotate(args.input, args.output, args.coordinate_system)


if __name__ == "__main__":
    raise SystemExit(main())
