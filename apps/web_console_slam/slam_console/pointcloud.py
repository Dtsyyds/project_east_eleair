"""Read-only PointCloud2 preview, including organized clouds and byte order."""
import numpy as np


def xyz_preview(message, limit=2000):
    if not 1 <= limit <= 2000:
        raise ValueError('点数上限须在 1..2000')
    width, height = int(message.width), int(message.height)
    if width < 0 or height < 0:
        raise ValueError('负点云尺寸')
    if not width or not height:
        return []
    step, row = int(message.point_step), int(message.row_step)
    if step <= 0 or row < width * step or len(message.data) < row * height:
        raise ValueError('PointCloud2 数据长度/行步长无效')
    fields = {f.name: f for f in message.fields}
    endian = '>' if message.is_bigendian else '<'
    columns = []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.datatype not in (7, 8) or field.count != 1:
            raise ValueError('XYZ 必须为单值 float32/float64')
        dtype = np.dtype(endian + ('f4' if field.datatype == 7 else 'f8'))
        if field.offset < 0 or field.offset + dtype.itemsize > step:
            raise ValueError('XYZ 字段越界')
        columns.append(np.ndarray((height, width), dtype=dtype, buffer=message.data,
                                  offset=field.offset, strides=(row, step)).reshape(-1))
    points = np.column_stack(columns)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) > limit:
        points = points[np.linspace(0, len(points) - 1, limit, dtype=int)]
    return points.tolist()
