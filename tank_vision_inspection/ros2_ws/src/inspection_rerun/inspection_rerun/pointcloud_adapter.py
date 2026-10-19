"""Minimal PointCloud2 XYZ/intensity parser with bounded downsampling."""

import numpy as np


_FIELD_DTYPES = {
    1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8",
}


def _field(message, name):
    return next((item for item in message.fields if item.name == name), None)


def _extract_field(message, field):
    if int(field.count) != 1 or int(field.datatype) not in _FIELD_DTYPES:
        raise ValueError(f"unsupported PointCloud2 field {field.name}")
    byte_order = ">" if bool(message.is_bigendian) else "<"
    dtype = np.dtype(byte_order + _FIELD_DTYPES[int(field.datatype)])
    width = int(message.width)
    height = int(message.height)
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    raw = memoryview(message.data)
    if raw.nbytes < height * row_step:
        raise ValueError("PointCloud2 data is shorter than height * row_step")
    rows = []
    for row in range(height):
        values = np.ndarray(
            shape=(width,), dtype=dtype, buffer=raw,
            offset=row * row_step + int(field.offset), strides=(point_step,),
        )
        rows.append(values)
    return np.concatenate(rows) if len(rows) > 1 else np.asarray(rows[0])


def pointcloud_to_numpy(
        message, point_stride=1, max_point_count=100000, voxel_size=0.0,
        return_stats=False):
    """Convert a PointCloud2 to bounded numpy arrays.

    ``return_stats`` keeps the historical two-value API intact while allowing
    the offline builder to report exactly how many points were rejected or
    capped.  Voxel filtering retains the first finite point in every occupied
    voxel and never mutates the source message.
    """
    stride = max(1, int(point_stride))
    limit = max(1, int(max_point_count))
    voxel = float(voxel_size)
    if voxel < 0.0:
        raise ValueError("voxel_size must be non-negative")
    xyz_fields = [_field(message, axis) for axis in ("x", "y", "z")]
    if any(field is None for field in xyz_fields):
        raise ValueError("PointCloud2 must contain x, y and z fields")
    xyz = np.column_stack([_extract_field(message, field) for field in xyz_fields]).astype(
        np.float32, copy=False)
    intensity_field = _field(message, "intensity")
    intensity = _extract_field(message, intensity_field).astype(np.float32, copy=False) if intensity_field else None
    valid = np.isfinite(xyz).all(axis=1)
    if intensity is not None:
        valid &= np.isfinite(intensity)
    valid_indices = np.flatnonzero(valid)
    indices = valid_indices[::stride]
    if voxel > 0.0 and indices.size:
        coordinates = np.floor(xyz[indices].astype(np.float64) / voxel).astype(np.int64)
        _, first = np.unique(coordinates, axis=0, return_index=True)
        indices = indices[np.sort(first)]
    before_cap = int(indices.size)
    if indices.size > limit:
        indices = indices[np.linspace(0, indices.size - 1, limit, dtype=np.int64)]
    result = (
        np.ascontiguousarray(xyz[indices]),
        np.ascontiguousarray(intensity[indices]) if intensity is not None else None,
    )
    if not return_stats:
        return result
    statistics = {
        "source_point_count": int(xyz.shape[0]),
        "invalid_point_count": int(xyz.shape[0] - valid_indices.size),
        "stride_removed_point_count": int(valid_indices.size - valid_indices[::stride].size),
        "voxel_removed_point_count": int(valid_indices[::stride].size - before_cap),
        "capped_point_count": int(before_cap - indices.size),
        "output_point_count": int(indices.size),
    }
    return result[0], result[1], statistics
