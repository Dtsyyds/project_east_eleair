"""Safe result JSON writing."""

from __future__ import annotations

import json
from pathlib import Path

from tank_vision.types import VisionResult


def serialize_result(result: VisionResult) -> str:
    return json.dumps(result.to_dict(), ensure_ascii=False, indent=2, allow_nan=False)


def write_json(result: VisionResult, path: str | Path) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(serialize_result(result), encoding="utf-8")
