"""A/center decoder plus explicit operator-selected recovery, candidate mode only."""
import time
from dataclasses import asdict
import cv2
from .round1_a import Round1APipeline
from .learning import preprocess
from .center_2d import decode
from .geometry import roi_bounds
from .recovery import RecoveryTracker
from .single_boundary import SingleBoundary

class CandidateVisual:
    def __init__(self, original):
        from .inference_backend import CPUInference
        self.original=original
        self.model_sha256=original.model_sha256
        self.last_transform=None
        self.backend=CPUInference(original.model,'cpu_channels_last')
    def infer(self,image):
        x,transform=preprocess(image,self.original.size,rgb=True)
        self.last_transform=asdict(transform)
        probability=self.backend.probability(x)
        return cv2.resize(probability,(image.shape[1],image.shape[0]),interpolation=cv2.INTER_LINEAR),'round1_a_rgb_cpu_channels_last',None
    def close(self):self.original.close()

class CandidatePipeline(Round1APipeline):
    def __init__(self,config):
        super().__init__(config)
        self.visual=CandidateVisual(self.visual)
        self.frame_decision=None
        self.recovery=RecoveryTracker(target_loss_sec=config.get("selected_target_memory_sec", 5.))
        self.single_boundary=SingleBoundary(config)
    def decode_visual(self,probability,config,bounds):
        self.frame_decision=decode(probability,config,bounds,enabled=True)
        return self.frame_decision['A']
    def process(self,frame,diagnostic=False):
        started=time.perf_counter();self.frame_decision=None
        self.single_boundary.begin(frame)
        result,details=super().process(frame,diagnostic=True)
        d=self.frame_decision
        result['runtime_mode']='candidate'
        result['measurement_kind']='A_COMPLETE' if result['valid_2d'] else 'NONE'
        if d and d['kind']=='CENTER_2D':
            result.update(state='CENTER_2D',valid_2d=True,metric_valid=False,
                predicted_only=False,measurement_kind='CENTER_2D',center_2d_valid=True,
                points_2d_y_left_right=[],points_2d_y_center=d['center_points'],
                confidence=float(d['A']['confidence'])*len(d['center_points'])/96,
                evidence_source=['current_frame_measured_center_only'],
                A_rejection_reasons=list(d['A']['reasons']),reasons=[])
        anchor = self.recovery.anchor if self.recovery.manual else None
        if not self.recovery.manual or anchor is not None:
            self.single_boundary.learn(result, frame.image_stamp_ns, anchor)
            if not result['valid_2d']:
                single = self.single_boundary.infer(details['probability'], roi_bounds(frame.image.shape, self.config),
                                                    frame.image_stamp_ns, anchor)
                if single:
                    result.update(state='CENTER_2D', valid_2d=True, center_2d_valid=True,
                        metric_valid=False, predicted_only=False, measurement_kind='SINGLE_BOUNDARY_2D',
                        points_2d_y_left_right=[], points_2d_y_center=single['points'],
                        confidence=single['confidence'], single_boundary=single,
                        A_rejection_reasons=list(result['visual']['reasons']), reasons=[],
                        evidence_source=['current_frame_single_boundary_with_width'])
        result=self.recovery.process(frame,result,details['probability'],self.config,roi_bounds(frame.image.shape,self.config))
        result['timing_ms']['total']=(time.perf_counter()-started)*1000
        return (result,details) if diagnostic else result
