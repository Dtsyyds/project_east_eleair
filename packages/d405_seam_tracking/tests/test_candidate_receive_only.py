import math
import numpy as np
from d405_seam_tracking.receiver_hold import ErrorHold
from d405_seam_tracking.tracking_error import ErrorReference, tracking_error

def msg(seq,valid=True):
    return dict(frame_seq=seq,stamp_ns=1000000000+seq*100000000,valid=valid,lateral_error_norm=-.1,heading_error_rad=.03,confidence=.9)

def test_hold_four_fifth_invalid_and_recover():
    h=ErrorHold();first=h.receive(msg(1),0)
    for i in range(1,5):
        r=h.receive(msg(i+1,False),i*.1)
        assert r['state']=='HOLD' and r['measurement_stamp_ns']==first['measurement_stamp_ns']
        assert r['consecutive_invalid_processed_frames']==i and not r['new_detection']
    r=h.receive(msg(6,False),.5);assert r['state']=='LOST' and r['lateral_error_norm'] is None
    assert h.receive(msg(7,False),.6)['state']=='LOST'
    assert h.receive(msg(8),.7)['state']=='MEASURED'
    assert h.misses==0

def test_no_history_duplicates_timeouts_and_source_loss():
    h=ErrorHold();assert h.receive(msg(1,False),0)['state']=='LOST'
    h.receive(msg(2),.1);h.receive(msg(3,False),.2)
    assert h.receive(msg(3,False),.3)['consecutive_invalid_processed_frames']==1
    assert h.tick(.71)['state']=='LOST'
    assert h.receive(msg(4,False),.72)['state']=='LOST'
    h.receive(msg(5),.8);assert h.source_alive(False)['state']=='LOST'
    assert h.receive(msg(6),.9)['state']=='LOST'
    h.source_alive(True);assert h.receive(msg(7),1)['state']=='MEASURED'

def test_missing_message_or_changed_publisher_cannot_hold():
    h=ErrorHold();h.receive(msg(1),0,'a')
    assert h.receive(msg(3,False),.1,'a')['state']=='LOST'
    h.receive(msg(4),.2,'a')
    assert h.receive(msg(1,False),.3,'b')['state']=='LOST'

def test_center_only_error_same_reference_and_no_boundaries():
    y=np.arange(168,456,3);x=288+math.tan(math.radians(3))*(y-360)
    r=dict(state='CENTER_2D',center_2d_valid=True,valid_2d=True,predicted_only=False,
           coordinate_2d='original_image_pixel_centers',confidence=.8,points_2d_y_center=np.column_stack((y,x)).tolist(),points_2d_y_left_right=[])
    e,reason=tracking_error(r,640,480,ErrorReference())
    assert reason is None and e['valid']
    assert abs(e['lateral_error_norm']+.1)<1e-12 and abs(e['heading_error_rad']-math.radians(3))<1e-12
    r['points_2d_y_center']=r['points_2d_y_center'][:55]
    assert tracking_error(r,640,480,ErrorReference())[0]['valid'] is False
