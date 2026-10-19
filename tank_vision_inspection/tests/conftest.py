from __future__ import annotations

from pathlib import Path

import pytest

from tank_vision.config import AppConfig, load_config


@pytest.fixture
def config() -> AppConfig:
    path = Path(__file__).parents[1] / "configs" / "default.yaml"
    result = load_config(path)
    result.pipeline.thumbnail_max_dimension = 0
    return result

