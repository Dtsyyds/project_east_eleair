from __future__ import annotations

import numpy as np

from tank_vision.defects.tiled_inference import TiledInference
from tank_vision.types import DefectResult


def test_edge_tiles_are_padded_and_cover_image() -> None:
    image = np.zeros((1300, 1500, 3), dtype=np.uint8)
    tiler = TiledInference(1024, 128)
    tiles = tiler.tiles(image)
    assert len(tiles) == 4
    assert all(tile.image.shape[:2] == (1024, 1024) for tile in tiles)
    assert max(tile.x + tile.valid_width for tile in tiles) == 1500
    assert max(tile.y + tile.valid_height for tile in tiles) == 1300


def test_detection_coordinates_restore_to_original() -> None:
    image = np.zeros((1300, 1500, 3), dtype=np.uint8)
    tile = TiledInference(1024, 128).tiles(image)[-1]
    local = DefectResult(
        "local",
        "surface_anomaly",
        0.8,
        (100, 120, 200, 220),
        [(100.0, 120.0), (200.0, 220.0)],
        1000.0,
        (150.0, 170.0),
        "test",
    )
    restored = TiledInference.restore_detection(local, tile, image.shape)
    assert restored is not None
    assert restored.bbox_xyxy == (tile.x + 100, tile.y + 120, tile.x + 200, tile.y + 220)
    assert restored.centroid_uv == (tile.x + 150.0, tile.y + 170.0)


def test_small_image_uses_padded_single_tile() -> None:
    tile = TiledInference(1024, 128).tiles(np.zeros((100, 200, 3), dtype=np.uint8))[0]
    assert tile.image.shape == (1024, 1024, 3)
    assert (tile.valid_width, tile.valid_height) == (200, 100)

