import struct
import numpy as np

from inspection_rerun.pointcloud_adapter import pointcloud_to_numpy


class Field:
    def __init__(self, name, offset, datatype=7, count=1):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, count


class Cloud:
    pass


def make_cloud(points):
    msg = Cloud()
    msg.width, msg.height = len(points), 1
    msg.point_step = 16
    msg.row_step = 16 * len(points)
    msg.is_bigendian = False
    msg.fields = [Field("x", 0), Field("y", 4), Field("z", 8), Field("intensity", 12)]
    msg.data = b"".join(struct.pack("<ffff", *point) for point in points)
    return msg


def test_extracts_xyz_and_intensity():
    xyz, intensity = pointcloud_to_numpy(make_cloud([(1, 2, 3, 4), (5, 6, 7, 8)]))
    np.testing.assert_allclose(xyz, [[1, 2, 3], [5, 6, 7]])
    np.testing.assert_allclose(intensity, [4, 8])


def test_filters_nan_and_limits_count():
    cloud = make_cloud([(1, 2, 3, 4), (float("nan"), 2, 3, 4), (5, 6, 7, 8), (9, 10, 11, 12)])
    xyz, intensity = pointcloud_to_numpy(cloud, point_stride=1, max_point_count=2)
    assert xyz.shape == (2, 3)
    assert intensity.shape == (2,)
    assert np.isfinite(xyz).all()


def test_xyz_without_intensity_is_supported():
    cloud = make_cloud([(1, 2, 3, 4)])
    cloud.fields = cloud.fields[:3]
    xyz, intensity = pointcloud_to_numpy(cloud)
    np.testing.assert_allclose(xyz, [[1, 2, 3]])
    assert intensity is None
