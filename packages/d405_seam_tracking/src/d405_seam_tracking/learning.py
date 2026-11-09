"""Single SMP MobileNetV2 U-Net. RGB/IR both become luminance, NCHW [0,1]."""
import json
import os
from pathlib import Path
import random
import time
from dataclasses import asdict
import cv2
import numpy as np
from .contracts import dump, sha256, PixelTransform
from .dataset import load_index, read_sample, annotation_mask, label_valid_region, split_check


def preprocess(image, size, rgb=False):
    if rgb:
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError('rgb_uint8_input_required')
        h, w = size
        x = cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA).astype(np.float32) / 255.
        x = (x - np.array([.485, .456, .406], np.float32)) / np.array([.229, .224, .225], np.float32)
        return np.ascontiguousarray(x.transpose(2, 0, 1)[None]), PixelTransform(0, 0, w / image.shape[1], h / image.shape[0])
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    h, w = size
    x = cv2.resize(gray, (w, h), interpolation=cv2.INTER_AREA).astype(np.float32) / 255.
    return x[None, None], PixelTransform(0, 0, w / image.shape[1], h / image.shape[0])


def build_model(in_channels=1, encoder_weights_path=None):
    os.environ.setdefault('HF_HUB_OFFLINE', '1')
    os.environ.setdefault('HF_HUB_DISABLE_TELEMETRY', '1')
    import segmentation_models_pytorch as smp
    if smp.__version__ != '0.5.0': raise RuntimeError('requires pinned segmentation_models_pytorch==0.5.0')
    model = smp.Unet(encoder_name='mobilenet_v2', encoder_weights=None, in_channels=in_channels, classes=1, activation=None,
                     decoder_channels=(128, 64, 32, 16, 8))
    if encoder_weights_path:
        if in_channels != 3:
            raise ValueError('pretrained_prototype_requires_rgb')
        from safetensors.torch import load_file
        state = load_file(str(encoder_weights_path))
        # SMP removes only the ImageNet classifier; all encoder tensors load strictly.
        model.encoder.load_state_dict(state, strict=True)
    return model


def training_labels(annotation, shape, ignore_boundary_pixels=0, allow_synthetic=False):
    mask, rows = annotation_mask(annotation, shape, allow_synthetic=allow_synthetic)
    labels = mask.astype(np.float32)
    labels[~label_valid_region(shape, rows)] = -1
    if ignore_boundary_pixels and rows:
        p = np.asarray(rows)
        xx = np.arange(shape[1])[None]
        uncertain = (abs(xx-p[:, 1, None]) <= ignore_boundary_pixels) | (abs(xx-p[:, 2, None]) <= ignore_boundary_pixels)
        yy = p[:, 0].astype(int)
        block = labels[yy].copy(); block[uncertain] = -1; labels[yy] = block
    return labels


def training_split_status(records, group_key='session'):
    if group_key == 'session':
        return split_check(records)
    if group_key != 'capture_segment':
        raise ValueError('unsupported_training_split_group')
    groups = {}
    for record in records:
        group = record.get(group_key)
        if not group or record['split'] not in ('train', 'val', 'test', 'unassigned'):
            raise ValueError('invalid_capture_segment_split')
        if groups.setdefault(group, record['split']) != record['split']:
            raise ValueError('capture_segment_leaks_across_splits')
    return dict(session_count=len({r['session'] for r in records}), group_key=group_key,
                group_count=len(groups), independent_generalization_sufficient=False)


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


def train(index_path, output, epochs=1, size=(128, 160), seed=17, resume=None, smoke=False, *,
          rgb=False, encoder_weights_path=None, split_group='session', ignore_boundary_pixels=0,
          freeze_encoder_epochs=0, learning_rate=1e-3, encoder_learning_rate=1e-4,
          horizontal_shift_fraction=0., log_progress=False):
    import torch
    import segmentation_models_pytorch as smp
    torch.set_num_threads(2)
    if epochs < 1: raise ValueError('positive_epochs_required')
    random.seed(seed); np.random.seed(seed); torch.manual_seed(seed)
    root, index = load_index(index_path)
    split_status = training_split_status(index['samples'], split_group)
    if rgb and not encoder_weights_path:
        raise ValueError('rgb_prototype_requires_explicit_pretrained_encoder')
    if (ignore_boundary_pixels < 0 or freeze_encoder_epochs < 0 or not 0 <= horizontal_shift_fraction <= .25):
        raise ValueError('invalid_training_options')
    if index.get('synthetic') and not smoke: raise ValueError('synthetic dataset requires explicit --smoke')
    samples = [r for r in index['samples'] if r['split'] == 'train' and (r.get('annotation') or {}).get('status') == 'approved']
    if not samples: raise ValueError('no approved training samples')
    if min(size) < 64 or any(s % 32 for s in size): raise ValueError('input dimensions must be multiples of 32 and >=64')
    target = Path(output); target.mkdir(parents=True, exist_ok=False)
    model = build_model(in_channels=3 if rgb else 1, encoder_weights_path=encoder_weights_path)
    if rgb:
        optimizer = torch.optim.Adam([{'params': model.encoder.parameters(), 'lr': encoder_learning_rate},
                                      {'params': list(model.decoder.parameters()) + list(model.segmentation_head.parameters()),
                                       'lr': learning_rate}])
    else:
        optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    start_epoch = 0
    history = []
    if resume:
        cp = torch.load(resume, map_location='cpu', weights_only=True)
        if cp['architecture'] != ('smp_unet_mobilenet_v2_rgb_imagenet' if rgb else 'smp_unet_mobilenet_v2_1channel'):
            raise ValueError('resume_architecture_mismatch')
        if tuple(cp['input_size']) != tuple(size): raise ValueError('resume_input_contract_mismatch')
        model.load_state_dict(cp['model']); optimizer.load_state_dict(cp['optimizer']); start_epoch = cp['epoch']
        history = cp['history']
    dice = smp.losses.DiceLoss(mode='binary', from_logits=True, ignore_index=-1)
    bce = smp.losses.SoftBCEWithLogitsLoss(pos_weight=torch.tensor([3.]), ignore_index=-1)
    # Only training/validation records are read here. Held-out annotations are not consumed.
    cached = {}
    if rgb:
        for record in index['samples']:
            if record['split'] in ('train', 'val') and record.get('annotation'):
                frame = read_sample(root, record)
                labels = training_labels(record['annotation'], frame.image.shape, ignore_boundary_pixels, smoke)
                cached[record['sample']] = (frame.image, labels)
    # Optional paired shifts are RGB training tensors only; raw/depth/calibration stay unchanged.
    for epoch in range(start_epoch, start_epoch + epochs):
        started = time.monotonic()
        model.train(); losses = []
        if rgb:
            model.encoder.requires_grad_(epoch >= freeze_encoder_epochs)
            model.encoder.eval()  # Keep pretrained BN statistics stable with single-image batches.
        shuffled = list(samples); random.shuffle(shuffled)
        for record in shuffled:
            if rgb:
                image, labels = cached[record['sample']]
                image = np.clip(image.astype(np.float32)*random.uniform(.85, 1.15)+255*random.uniform(-.05, .05), 0, 255).astype(np.uint8)
                if random.random() < .2: image = cv2.GaussianBlur(image, (3, 3), .7)
                if horizontal_shift_fraction:
                    shift = int(round(random.uniform(-horizontal_shift_fraction, horizontal_shift_fraction)*image.shape[1]))
                    matrix = np.array([[1, 0, shift], [0, 1, 0]], np.float32)
                    image = cv2.warpAffine(image, matrix, (image.shape[1], image.shape[0]), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_REFLECT_101)
                    labels = cv2.warpAffine(labels, matrix, (labels.shape[1], labels.shape[0]), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=-1)
                x, _ = preprocess(image, size, rgb=True)
            else:
                frame = read_sample(root, record)
                labels = training_labels(record['annotation'], frame.image.shape, ignore_boundary_pixels, smoke)
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
                if rgb:
                    image, labels = cached[record['sample']]
                    x, _ = preprocess(image, size, rgb=True)
                else:
                    frame = read_sample(root, record)
                    labels = training_labels(record['annotation'], frame.image.shape, ignore_boundary_pixels, smoke)
                    x, _ = preprocess(frame.image, size)
                y = cv2.resize(labels, (size[1], size[0]), interpolation=cv2.INTER_NEAREST)[None, None]
                logits = model(torch.from_numpy(x)); truth = torch.from_numpy(y)
                validation.append(float(bce(logits, truth) + dice(logits, truth)))
        history.append({'epoch': epoch + 1, 'train_loss': float(np.mean(losses)), 'val_loss': float(np.mean(validation)) if validation else None,
                        'seconds': time.monotonic()-started})
        if log_progress:
            print(json.dumps(dict(output=str(target), **history[-1])), flush=True)
            dump(target / 'progress.json', history)
    checkpoint = {'model': model.state_dict(), 'optimizer': optimizer.state_dict(), 'epoch': start_epoch + epochs,
                  'history': history, 'input_size': list(size),
                  'architecture': 'smp_unet_mobilenet_v2_rgb_imagenet' if rgb else 'smp_unet_mobilenet_v2_1channel',
                  'preprocessing': 'RGB; area_resize; uint8/255; ImageNet_mean_std; NCHW' if rgb else 'RGB_to_luminance_or_IR; area_resize; uint8/255; NCHW; no_NV12',
                  'encoder_weights': sha256(encoder_weights_path) if encoder_weights_path else None,
                  'dataset_sha256': sha256(index_path), 'seed': seed,
                  'training_samples': [r['sample'] for r in samples],
                  'synthetic_smoke_only': bool(smoke), 'trained_seam_model_ready': False}
    training_config = {'architecture': checkpoint['architecture'], 'input_size': list(size), 'seed': seed, 'epochs': epochs,
                       'optimizer': 'Adam', 'lr': learning_rate, 'pos_weight': 3., 'loss': 'SMP_SoftBCE_plus_Dice_ignore_-1',
                       'augmentation': 'brightness_contrast_light_blur', 'horizontal_shift_fraction': horizontal_shift_fraction,
                       'encoder_weights': checkpoint['encoder_weights'], 'encoder_lr': encoder_learning_rate if rgb else None,
                       'freeze_encoder_epochs': freeze_encoder_epochs, 'freeze_encoder_bn': rgb,
                       'split_group': split_group, 'ignore_boundary_pixels': ignore_boundary_pixels}
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
