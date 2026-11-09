"""Explicit RGB round-1 A adapter; the legacy Pipeline remains available.

The offline A measurement is frame-local. Temporal smoothing/prediction and
depth substitution would change that measurement, so this adapter never retains
a prior measurement. All candidate scores and quality gates are reused verbatim.
"""
from dataclasses import asdict
from pathlib import Path
import cv2
import numpy as np
from .contracts import sha256
from .learning import build_model, preprocess
from .pipeline import Pipeline


def project_file(value):
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    for root in Path(__file__).resolve().parents:
        if (root/'deploy/env.sh').is_file():
            return root/path
    raise RuntimeError('round1_a_project_root_unavailable')


class Round1AVisual:
    def __init__(self, config):
        import torch
        torch.set_num_threads(2)
        path = project_file(config['checkpoint'])
        self.model_sha256 = sha256(path)
        if self.model_sha256 != config['checkpoint_sha256']:
            raise ValueError('round1_a_checkpoint_sha256_mismatch')
        cp = torch.load(path, map_location='cpu', weights_only=True)
        if cp.get('architecture') != 'smp_unet_mobilenet_v2_rgb_imagenet':
            raise ValueError('round1_a_requires_rgb_imagenet_checkpoint')
        self.size = tuple(cp['input_size'])
        if self.size != (192, 256):
            raise ValueError('round1_a_input_size_mismatch')
        self.model = build_model(in_channels=3)
        self.model.load_state_dict(cp['model'], strict=True)
        self.model.eval()
        self.last_transform = None

    def infer(self, image):
        import torch
        x, transform = preprocess(image, self.size, rgb=True)
        self.last_transform = asdict(transform)
        with torch.inference_mode():
            probability = torch.sigmoid(self.model(torch.from_numpy(x))).numpy()[0, 0]
        return cv2.resize(probability, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_LINEAR), 'round1_a_rgb_cpu', None

    def close(self):
        pass


class Round1APipeline(Pipeline):
    def __init__(self, config):
        if config['mode'] != 'vision_only' or config['candidate_selection'] != 'continuous':
            raise ValueError('round1_a_requires_frame_local_visual_continuous_decode')
        # This only avoids the legacy single-channel loader during construction.
        super().__init__(dict(config, visual_backend='classical'))
        self.visual = Round1AVisual(config)

    def process(self, frame, diagnostic=False):
        self.last_measurement = None
        try:
            return super().process(frame, diagnostic=diagnostic)
        finally:
            self.last_measurement = None


def create_pipeline(config):
    algorithm = config.get('algorithm', 'legacy')
    if algorithm == 'round1_a':
        return Round1APipeline(config)
    if algorithm == 'legacy':
        return Pipeline(config)
    raise ValueError('unsupported_d405_algorithm:' + str(algorithm))
