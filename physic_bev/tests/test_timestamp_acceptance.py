"""Sensor timestamp acceptance rejects invalid headers instead of empty monotonicity."""
import importlib.util
from pathlib import Path
import sys
import pytest
ROOT=Path(__file__).parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import validate_vehicle_collection as validator

@pytest.mark.parametrize('stamp,field',[(0,'zero_header_count'),(-1,'negative_header_count'),(None,'missing_header_count')])
def test_dynamic_all_invalid_headers_fail(stamp,field):
    item=validator._empty_time_item('sensor_msgs/msg/Imu')
    item['count']=5
    for _ in range(5): validator._update_monotonic(item,stamp)
    assert item[field]==5 and item['header_stamp_count']==0
    assert not validator.timestamp_contract_valid('/livox/imu',item)


def test_duplicate_regression_and_intervals_are_separate():
    item=validator._empty_time_item('eddy_driver/msg/EddyChannelMean')
    item['count']=5
    for t in [10,20,20,15,30]: validator._update_monotonic(item,t)
    assert item['duplicate_header_count']==1
    assert item['header_stamp_regressions']==1
    assert item['interval_min_ns']==10 and item['interval_max_ns']==15
    assert not validator.timestamp_contract_valid('/eddy/channel_mean',item)


def test_headerless_and_static_explicit_exceptions():
    item=validator._empty_time_item('std_msgs/msg/String');item['count']=1
    validator._update_monotonic(item,None)
    assert validator.timestamp_contract_valid('/field/collection_status',item)
    item['type']='sensor_msgs/msg/Image'
    assert validator.timestamp_contract_valid('/tf_static',item)
    assert not validator.timestamp_contract_valid('/camera/front/image_raw',item)
