"""CustomMsg validation, preflight freshness and preview compatibility."""
import copy
import threading
from types import SimpleNamespace

import pytest
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2

from inspection_operations.operations_node import OperationsNode
from inspection_operations.replay_worker import sensor_preview
from inspection_tools.validators import validate_livox_custom
from inspection_runtime.operations import Operations, DATA_TIMEOUT_SECONDS


def cloud():
    msg = CustomMsg(timebase=1800000000000000123, point_num=3,
                    points=[CustomPoint(x=1., y=2., z=3., offset_time=123, reflectivity=50, tag=16, line=2),
                            CustomPoint(x=float('nan')), CustomPoint(z=float('inf'))])
    msg.header.stamp.sec = 1800000000
    msg.header.stamp.nanosec = 123
    msg.header.frame_id = 'livox_frame'
    return msg


@pytest.mark.parametrize('field,value', [('point_num', 0), ('point_num', 4), ('timebase', 0),
                                        ('stamp', (0, 0)), ('stamp', (-1, 5)), ('stamp', (1, 10**9))])
def test_invalid_structure_and_stamp(field, value):
    msg = cloud()
    assert validate_livox_custom(msg) == []  # NaN/Inf are retained in the raw record.
    if field == 'stamp':
        msg.header.stamp.sec, msg.header.stamp.nanosec = value
    else:
        setattr(msg, field, value)
    assert validate_livox_custom(msg)
    adapter = SimpleNamespace(live={'/livox/lidar': 42.}, live_lock=threading.Lock())
    OperationsNode.seen(adapter, '/livox/lidar', msg)
    assert adapter.live['/livox/lidar'] == 42.
    with pytest.raises(ValueError, match='invalid_livox_custom_msg'):
        sensor_preview('lidar', msg)


def test_custom_preview_preserves_raw_and_bounds_display():
    msg = cloud()
    original = serialize_message(msg)
    result = sensor_preview('lidar', msg)
    assert result['xyz'] == [1., 2., 3.]
    assert result['point_count'] == 3 and result['displayed_points'] == 1
    assert result['timebase'] == '1800000000000000123'
    assert serialize_message(msg) == original
    msg.points = [CustomPoint(x=float(i)) for i in range(10001)]
    msg.point_num = len(msg.points)
    result = sensor_preview('lidar', msg)
    assert result['displayed_points'] == 3334
    assert result['xyz'][::3] == list(range(0, 10001, 3))
    with pytest.raises(ValueError, match='unsupported lidar message type'):
        sensor_preview('lidar', object())


def test_only_valid_custom_frames_refresh_collection_preflight(monkeypatch):
    clock = [100.]
    monkeypatch.setattr('inspection_operations.operations_node.time.monotonic', lambda: clock[0])
    adapter = SimpleNamespace(live={}, live_lock=threading.Lock())
    operation = SimpleNamespace(
        storage=lambda: {'mounted': True, 'free_bytes': 100},
        config={'minimum_free_bytes': 1, 'required_topics': ['/livox/lidar']},
        live_topics=lambda: dict(adapter.live), lock=threading.Lock(), task={})
    with pytest.raises(RuntimeError, match='/livox/lidar'):
        Operations.preflight(operation)
    good = cloud()
    OperationsNode.seen(adapter, '/livox/lidar', good)
    Operations.preflight(operation)
    for i in range(1, int(DATA_TIMEOUT_SECONDS) + 3):
        clock[0] = 100. + i
        bad = copy.deepcopy(good); bad.timebase = 0
        OperationsNode.seen(adapter, '/livox/lidar', bad)
        OperationsNode.seen(adapter, '/livox/lidar', PointCloud2())
    assert adapter.live['/livox/lidar'] == 100.
    with pytest.raises(RuntimeError, match='/livox/lidar'):
        Operations.preflight(operation)
    # A positive subsecond header is valid too.
    good.header.stamp.sec = 0
    OperationsNode.seen(adapter, '/livox/lidar', good)
    Operations.preflight(operation)
    operation.config['required_topics'].append('/camera/front/image_raw')
    with pytest.raises(RuntimeError, match='/camera/front/image_raw'):
        Operations.preflight(operation)
