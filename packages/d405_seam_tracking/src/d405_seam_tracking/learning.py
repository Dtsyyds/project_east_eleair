"""Single SMP MobileNetV2 U-Net. RGB/IR both become luminance, NCHW [0,1]."""
import json
import os
from pathlib import Path
import random
from dataclasses import asdict
import cv2
import numpy as np
from .contracts import dump, sha256, PixelTransform
from .dataset import load_index, read_sample, annotation_mask, label_valid_region, split_check


def preprocess(image, size):
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    h, w = size
    x = cv2.resize(gray, (w, h), interpolation=cv2.INTER_AREA).astype(np.float32) / 255.
    return x[None, None], PixelTransform(0, 0, w / image.shape[1], h / image.shape[0])


def build_model():
    os.environ.setdefault('HF_HUB_OFFLINE', '1')
    os.environ.setdefault('HF_HUB_DISABLE_TELEMETRY', '1')
    import segmentation_models_pytorch as smp
    if smp.__version__ != '0.5.0': raise RuntimeError('requires pinned segmentation_models_pytorch==0.5.0')
    return smp.Unet(encoder_name='mobilenet_v2', encoder_weights=None, in_channels=1, classes=1, activation=None,
                    decoder_channels=(128, 64, 32, 16, 8))


class VisualModel:
    def __init__(self, config):
        self.config, self.model, self.bpu = config, None, None
        self.fallback = None
        self.model_sha256 = None
        self.last_transform = None
        backend = config['visual_backend']
        path = config.get('checkpoint')
        if backend == 'bpu':
            from .acceleration import BPUVisual
            self.bpu = BPUVisual(config.get('hbm'), config.get('hbm_contract'))
            self.model_sha256 = sha256(config['hbm'])
        elif path and Path(path).is_file():
            import torch
            torch.set_num_threads(2)
            checkpoint = torch.load(path, map_location='cpu', weights_only=True)
            if checkpoint.get('architecture') != 'smp_unet_mobilenet_v2_1channel': raise ValueError('checkpoint_architecture_mismatch')
            self.model_sha256 = sha256(path)
            self.size = tuple(checkpoint['input_size'])
            self.model = build_model(); self.model.load_state_dict(checkpoint['model']); self.model.eval()
        elif backend == 'auto': self.fallback = 'model_unavailable_fallback_classical'
        else: raise RuntimeError('model_unavailable: explicit SMP checkpoint required; no downloads')

    def infer(self, image):
        if self.bpu:
            probability = self.bpu.infer(image)
            self.last_transform = self.bpu.last_transform
            return probability, 'bpu_hbm', None
        if self.model is None:
            from .geometry import classical_probability
            return classical_probability(image, self.config), 'classical_cpu', self.fallback
        import torch
        x, transform = preprocess(image, self.size)
        self.last_transform = asdict(transform)
        with torch.inference_mode(): probability = torch.sigmoid(self.model(torch.from_numpy(x))).numpy()[0, 0]
        return cv2.resize(probability, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_LINEAR), 'smp_cpu', None

    def close(self):
        if self.bpu: self.bpu.close()


def train(index_path, output, epochs=1, size=(128, 160), seed=17, resume=None, smoke=False):
    import torch
    import segmentation_models_pytorch as smp
    torch.set_num_threads(2)
    if epochs < 1: raise ValueError('positive_epochs_required')
    random.seed(seed); np.random.seed(seed); torch.manual_seed(seed)
    root, index = load_index(index_path)
    split_status = split_check(index['samples'])
    if index.get('synthetic') and not smoke: raise ValueError('synthetic dataset requires explicit --smoke')
    samples = [r for r in index['samples'] if r['split'] == 'train' and (r.get('annotation') or {}).get('status') == 'approved']
    if not samples: raise ValueError('no approved training samples')
    if min(size) < 64 or any(s % 32 for s in size): raise ValueError('input dimensions must be multiples of 32 and >=64')
    target = Path(output); target.mkdir(parents=True, exist_ok=False)
    model = build_model(); optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    start_epoch = 0
    history = []
    if resume:
        cp = torch.load(resume, map_location='cpu', weights_only=True)
        if tuple(cp['input_size']) != tuple(size): raise ValueError('resume_input_contract_mismatch')
        model.load_state_dict(cp['model']); optimizer.load_state_dict(cp['optimizer']); start_epoch = cp['epoch']
        history = cp['history']
    dice = smp.losses.DiceLoss(mode='binary', from_logits=True, ignore_index=-1)
    bce = smp.losses.SoftBCEWithLogitsLoss(pos_weight=torch.tensor([3.]), ignore_index=-1)
    # No geometric augmentation: labels/depth/calibration stay in original coordinates.
    for epoch in range(start_epoch, start_epoch + epochs):
        model.train(); losses = []
        shuffled = list(samples); random.shuffle(shuffled)
        for record in shuffled:
            frame = read_sample(root, record)
            mask, rows = annotation_mask(record['annotation'], frame.image.shape, allow_synthetic=smoke)
            labels = mask.astype(np.float32)
            labels[~label_valid_region(frame.image.shape, rows)] = -1
            x, _ = preprocess(frame.image, size)
            x = np.clip(x * random.uniform(.85, 1.15) + random.uniform(-.05, .05), 0, 1)
            if random.random() < .2: x[0, 0] = cv2.GaussianBlur(x[0, 0], (3, 3), .7)
            y = cv2.resize(labels, (size[1], size[0]), interpolation=cv2.INTER_NEAREST)[None, None]
            optimizer.zero_grad(); logits = model(torch.from_numpy(x)); truth = torch.from_numpy(y)
            loss = bce(logits, truth) + dice(logits, truth)
            loss.backward(); optimizer.step(); losses.append(float(loss.detach()))
        model.eval(); validation = []
        with torch.inference_mode():
            for record in index['samples']:
                if record['split'] != 'val' or not record.get('annotation') or record['annotation'].get('status') != 'approved': continue
                frame = read_sample(root, record); mask, rows = annotation_mask(record['annotation'], frame.image.shape, allow_synthetic=smoke)
                labels = mask.astype(np.float32)
                labels[~label_valid_region(frame.image.shape, rows)] = -1
                x, _ = preprocess(frame.image, size)
                y = cv2.resize(labels, (size[1], size[0]), interpolation=cv2.INTER_NEAREST)[None, None]
                logits = model(torch.from_numpy(x)); truth = torch.from_numpy(y)
                validation.append(float(bce(logits, truth) + dice(logits, truth)))
        history.append({'epoch': epoch + 1, 'train_loss': float(np.mean(losses)), 'val_loss': float(np.mean(validation)) if validation else None})
    checkpoint = {'model': model.state_dict(), 'optimizer': optimizer.state_dict(), 'epoch': start_epoch + epochs,
                  'history': history, 'input_size': list(size), 'architecture': 'smp_unet_mobilenet_v2_1channel',
                  'preprocessing': 'RGB_to_luminance_or_IR; area_resize; uint8/255; NCHW; no_NV12',
                  'encoder_weights': None, 'dataset_sha256': sha256(index_path), 'seed': seed,
                  'synthetic_smoke_only': bool(smoke), 'trained_seam_model_ready': False}
    training_config = {'architecture': checkpoint['architecture'], 'input_size': list(size), 'seed': seed, 'epochs': epochs,
                       'optimizer': 'Adam', 'lr': 1e-3, 'pos_weight': 3., 'loss': 'SMP_SoftBCE_plus_Dice_ignore_-1',
                       'augmentation': 'brightness_contrast_light_blur_no_geometric', 'encoder_weights': None}
    import hashlib
    checkpoint['training_config'] = training_config
    checkpoint['training_config_sha256'] = hashlib.sha256(json.dumps(training_config, sort_keys=True).encode()).hexdigest()
    torch.save(checkpoint, target / 'checkpoint.pt')
    dump(target / 'history.json', {'history': history, 'split': split_status, 'checkpoint_sha256': sha256(target / 'checkpoint.pt'),
                                 'dataset_sha256': sha256(index_path), 'training_config': training_config,
                                 'training_config_sha256': checkpoint['training_config_sha256'], 'input_size': list(size), 'seed': seed, 'smoke_only': smoke})
    return target / 'checkpoint.pt'


def export_onnx(checkpoint, output):
    import torch
    import onnxruntime as ort
    torch.set_num_threads(2)
    if Path(output).exists() or Path(str(output) + '.json').exists(): raise FileExistsError(output)
    cp = torch.load(checkpoint, map_location='cpu', weights_only=True)
    model = build_model(); model.load_state_dict(cp['model']); model.eval()
    h, w = cp['input_size']
    x = torch.linspace(0, 1, h * w).reshape(1, 1, h, w)
    with torch.inference_mode(): expected = model(x).numpy()
    torch.onnx.export(model, x, output, opset_version=13, input_names=['luminance'], output_names=['seam_logits'], do_constant_folding=True)
    opt = ort.SessionOptions(); opt.intra_op_num_threads = 2
    session = ort.InferenceSession(str(output), sess_options=opt, providers=['CPUExecutionProvider'])
    actual = session.run(None, {'luminance': x.numpy()})[0]
    error = float(np.max(abs(expected - actual)))
    if not np.allclose(actual, expected, atol=1e-4, rtol=1e-4): raise ValueError('onnx_float_mismatch')
    report = {'onnx_sha256': sha256(output), 'checkpoint_sha256': sha256(checkpoint), 'input_shape': list(x.shape),
              'output_shape': list(actual.shape), 'output_semantics': 'binary seam logits; apply sigmoid exactly once',
              'max_abs_error': error, 'opset': 13, 'bpu_compilation_verified': False, 'target': 'CPU floating ONNX'}
    dump(str(output) + '.json', report)
    return report
