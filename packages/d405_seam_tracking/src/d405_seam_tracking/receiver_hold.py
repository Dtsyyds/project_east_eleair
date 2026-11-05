"""Receive-only error state; no ROS publisher or movement operation in this module."""
import math
import copy

class ErrorHold:
    def __init__(self,timeout_sec=.5):
        if timeout_sec<=0:raise ValueError('invalid timeout')
        self.timeout=timeout_sec;self.last_receipt=None;self.last_stamp=None;self.last_seq=None
        self.publisher=None;self.last_measurement=None;self.misses=0;self.alive=None
        self.state='LOST';self.reason='no_history';self.new_detection=False
    def invalidate(self,reason):
        self.last_measurement=None;self.state='LOST';self.reason=reason;self.new_detection=False
    def source_alive(self,value):
        self.alive=bool(value)
        if not self.alive:self.invalidate('source_stream_lost')
        return self.snapshot()
    def tick(self,now):
        self.new_detection=False
        if self.last_receipt is not None and now-self.last_receipt>=self.timeout:self.invalidate('message_timeout')
        return self.snapshot()
    def receive(self,message,now,publisher='unknown'):
        self.tick(now)
        seq=int(message['frame_seq']);stamp=int(message['stamp_ns'])
        self.new_detection=False
        if self.publisher is not None and publisher!=self.publisher:
            self.invalidate('publisher_changed');self.last_stamp=None;self.last_seq=None;self.misses=0
        self.publisher=publisher
        if stamp<=0:
            self.invalidate('invalid_source_stamp');return self.snapshot()
        if self.last_stamp is not None and stamp<=self.last_stamp:
            # Duplicates/stale packets do not extend the timeout or count as new frames.
            return self.snapshot()
        if self.last_seq is not None and seq!=self.last_seq+1:
            self.invalidate('sequence_discontinuity');self.misses=0
        self.last_stamp=stamp;self.last_seq=seq;self.last_receipt=now
        valid=bool(message['valid']) and all(math.isfinite(float(message[k])) for k in ['lateral_error_norm','heading_error_rad','confidence']) and 0<float(message['confidence'])<=1
        if self.alive is False:
            self.invalidate('source_stream_lost');return self.snapshot()
        if valid:
            self.last_measurement=copy.deepcopy(message);self.misses=0;self.state='MEASURED';self.reason=None;self.new_detection=True
        else:
            self.misses+=1
            if self.last_measurement is not None and self.misses<=4:
                self.state='HOLD';self.reason='temporary_hold'
            else:self.invalidate('five_invalid_frames' if self.misses>=5 else 'no_history')
        return self.snapshot()
    def snapshot(self):
        m=self.last_measurement
        return {'state':self.state,'state_label':{'MEASURED':'实测','HOLD':'暂持','LOST':'丢线'}[self.state],
                'valid':self.state in ('MEASURED','HOLD') and m is not None,'new_detection':self.new_detection,
                'consecutive_invalid_processed_frames':self.misses,'reason':self.reason,
                'measurement_stamp_ns':m['stamp_ns'] if m else None,'measurement_frame_seq':m['frame_seq'] if m else None,
                'lateral_error_norm':m['lateral_error_norm'] if m else None,'heading_error_rad':m['heading_error_rad'] if m else None,
                'confidence':m['confidence'] if m else 0.0}
