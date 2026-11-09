from __future__ import annotations
from inspection_runtime.paths import project_root, config_path, runtime_root, vision_evidence_root

from pathlib import Path

import pytest

from tank_vision.config import AppConfig, load_config


@pytest.fixture
def config() -> AppConfig:
    path = project_root() / 'config/vision/default.yaml'
    result = load_config(path)
    result.pipeline.thumbnail_max_dimension = 0
    return result

