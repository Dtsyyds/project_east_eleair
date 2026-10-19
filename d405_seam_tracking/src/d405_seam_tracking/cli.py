import argparse
from dataclasses import replace
import importlib.metadata
import json
from pathlib import Path
import platform
import time
import cv2
import numpy as np
from .contracts import Calibration, Frame, dump, load_config, sha256
from .dataset import load_index, read_sample, save_sample, annotation_mask, split_check
from .pipeline import Pipeline, overlay


def synthetic_frame(kind='bump', shape=(120, 160), stamp=1_000_000_000, sequence=0):
    h, w = shape
    c = Calibration(w, h, [w, 0, w / 2, 0, w, h / 2, 0, 0, 1], [], 'none', 'synthetic_optical', 'synthetic_factory_fixture')
    image = np.full((h, w), 70, np.uint8)
    yy, xx = np.mgrid[:h, :w]
    z = .3 + .004 * ((xx - w / 2) / w) ** 2 + .001 * (yy - h / 2) / h
    left, right = int(w * .465), int(w * .535)
    if kind not in ('none', 'geometry_only'): image[:, left:right + 1] = 200
    if kind not in ('none', 'flat_visual'): z[:, left:right + 1] -= .004 if kind != 'groove' else -.004
    depth = np.rint(z * 1000).astype(np.uint16)
    if kind == 'holes': depth[:, left:right + 1] = 0
    if kind == 'invalid': depth[:] = 0
    if kind == 'conflict':
        depth = np.full(shape, 300, np.uint16); depth[:, int(w * .7):int(w * .76)] = 295
    return Frame(image, depth, depth > 0, stamp, stamp, sequence, c, c,
                 {'rotation': np.eye(3).ravel().tolist(), 'translation_m': [0, 0, 0], 'from_frame': c.frame_id, 'to_frame': c.frame_id, 'source': 'synthetic'},
                 .001, {'synthetic': True, 'case': kind, 'session': 'synthetic'})


def fixture(output):
    root = Path(output); root.mkdir(parents=True, exist_ok=False)
    records = []
    for si, split in enumerate(('train', 'val', 'test')):
        for n, kind in enumerate(('bump', 'none', 'groove', 'flat_visual', 'invalid', 'geometry_only', 'holes', 'conflict')):
            f = synthetic_frame(kind, stamp=1_000_000_000 + (si * 8 + n) * 100_000_000, sequence=si * 8 + n)
            h, w = f.image.shape
            annotation = {'status': 'approved', 'author': 'synthetic_fixture_not_human', 'origin': 'synthetic',
                          'no_seam': kind in ('none', 'geometry_only'), 'left_xy': [[int(w * .465), 0], [int(w * .465), h - 1]],
                          'right_xy': [[int(w * .535), 0], [int(w * .535), h - 1]], 'interpolation': 'linear_in_original_y'}
            records.append(save_sample(root, f, f'fixture_{si}_{n}', 'synthetic_' + split, split, annotation))
    dump(root / 'index.json', {'schema': 'd405_dataset_v1', 'synthetic': True, 'sources': [], 'samples': records})
    return {'frames': len(records), 'synthetic_only': True}


def preflight():
    versions = {}
    for name in ('numpy', 'opencv-python', 'PyYAML', 'torch', 'torchvision', 'segmentation-models-pytorch', 'onnx', 'onnxruntime', 'hbm-runtime'):
        try: versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError: versions[name] = 'unavailable'
    gpu = {'status': 'unavailable'}
    try:
        from .acceleration import OpenCLGeometry
        g = OpenCLGeometry(); gpu = {'status': 'initialized_not_algorithm_verified', 'device': g.device_name}; g.close()
    except Exception as exc: gpu['reason'] = str(exc)
    return {'platform': platform.platform(), 'python': platform.python_version(), 'dependencies': versions, 'gpu': gpu,
            'seam_hbm': 'unavailable', 'real_seam_dataset_validated': False, 'production_ready': False}


def replay(index_path, config_path, output, modes, visual_backend=None, checkpoint=None, overlays=False, split_selection='all'):
    from .evaluation import Evaluator
    root, index = load_index(index_path)
    split = split_check(index['samples'])
    selected = [r for r in index['samples'] if split_selection == 'all' or r['split'] == split_selection]
    if not selected: raise ValueError('no_samples_in_selected_split')
    target = Path(output); target.mkdir(parents=True, exist_ok=False)
    base = load_config(config_path)
    if visual_backend: base['visual_backend'] = visual_backend
    if checkpoint: base['checkpoint'] = checkpoint
    summaries = {}
    for mode in modes:
        config = dict(base, mode=mode)
        pipe, evaluation = Pipeline(config), Evaluator()
        if overlays: (target / mode).mkdir()
        try:
            with (target / (mode + '.jsonl')).open('x') as stream:
                for record in selected:
                    frame = read_sample(root, record)
                    result, diagnostic = pipe.process(frame, diagnostic=True)
                    stream.write(json.dumps(result, ensure_ascii=False, allow_nan=False) + '\n')
                    evaluation.add(frame, result, record.get('annotation'), diagnostic['probability'])
                    if overlays: cv2.imwrite(str(target / mode / (str(frame.sequence) + '.jpg')), overlay(frame, result, diagnostic))
            summaries[mode] = evaluation.report()
        finally: pipe.close()
    summary = {'schema': 'd405_analysis_v1', 'source_index': str(Path(index_path).resolve()), 'source_index_sha256': sha256(index_path),
               'config_sha256': sha256(config_path), 'config': base, 'model_sha256': sha256(checkpoint) if checkpoint else None,
               'synthetic_only': bool(index.get('synthetic')), 'session_split': split, 'modes': summaries,
               'selected_split': split_selection,
               'source_run_id': index.get('source_run_id', 'unavailable'), 'calibration_status': index.get('calibration_status', 'per_frame'),
               'code_version': importlib.metadata.version('d405-seam-tracking'), 'algorithm_accuracy_validated': False, 'production_ready': False,
               'outputs': [{'path': p.name, 'sha256': sha256(p)} for p in sorted(target.glob('*.jsonl'))]}
    dump(target / 'result_manifest.json', summary)
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description='Independent D405 experiment, never starts camera or motion')
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('preflight')
    f = commands.add_parser('fixture'); f.add_argument('--output', required=True)
    for name in ('replay', 'evaluate'):
        p = commands.add_parser(name); p.add_argument('--index', required=True); p.add_argument('--config', required=True); p.add_argument('--output', required=True)
        p.add_argument('--mode', choices=['vision_only', 'depth_only', 'fused', 'all'], default='all')
        p.add_argument('--visual-backend', choices=['classical', 'smp', 'auto', 'bpu']); p.add_argument('--checkpoint'); p.add_argument('--overlays', action='store_true')
        p.add_argument('--split', choices=['all', 'train', 'val', 'test', 'unassigned'], default='all')
    p = commands.add_parser('train'); p.add_argument('--index', required=True); p.add_argument('--output', required=True); p.add_argument('--epochs', type=int, default=1)
    p.add_argument('--size', type=int, nargs=2, default=[128, 160]); p.add_argument('--seed', type=int, default=17); p.add_argument('--resume'); p.add_argument('--smoke', action='store_true')
    p = commands.add_parser('export'); p.add_argument('--checkpoint', required=True); p.add_argument('--output', required=True)
    p = commands.add_parser('extract'); p.add_argument('--bag', action='append', required=True); p.add_argument('--output', required=True); p.add_argument('--session', required=True)
    p.add_argument('--image-topic', required=True); p.add_argument('--depth-topic', required=True); p.add_argument('--image-info-topic', required=True); p.add_argument('--depth-info-topic', required=True); p.add_argument('--extrinsics-topic', required=True)
    p.add_argument('--depth-scale-m', type=float, required=True); p.add_argument('--sync-tolerance-ms', type=float, default=12.); p.add_argument('--start-sec', type=float, default=0.)
    p.add_argument('--duration-sec', type=float); p.add_argument('--every', type=int, default=1)
    p = commands.add_parser('annotate'); p.add_argument('--index', required=True); p.add_argument('--sample', required=True); p.add_argument('--annotation'); p.add_argument('--output', required=True)
    p.add_argument('--prepare', action='store_true', help='Export original image for the existing manual annotation GUI')
    p.add_argument('--approve', action='store_true'); p.add_argument('--author', help='Required approval author for legacy manual format')
    p = commands.add_parser('benchmark'); p.add_argument('--index', required=True); p.add_argument('--config', required=True); p.add_argument('--iterations', type=int, default=100); p.add_argument('--output', required=True)
    p = commands.add_parser('rerun'); p.add_argument('--index', required=True); p.add_argument('--results', required=True); p.add_argument('--output', required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == 'preflight': result = preflight()
        elif args.command == 'fixture': result = fixture(args.output)
        elif args.command in ('replay', 'evaluate'):
            modes = ['vision_only', 'depth_only', 'fused'] if args.mode == 'all' else [args.mode]
            result = replay(args.index, args.config, args.output, modes, args.visual_backend, args.checkpoint, args.overlays, args.split)
        elif args.command == 'train':
            from .learning import train
            result = {'checkpoint': str(train(args.index, args.output, args.epochs, tuple(args.size), args.seed, args.resume, args.smoke)), 'field_model_ready': False}
        elif args.command == 'export':
            from .learning import export_onnx
            result = export_onnx(args.checkpoint, args.output)
        elif args.command == 'extract':
            from .dataset import extract_mcap
            topics = {'image': args.image_topic, 'depth': args.depth_topic, 'image_info': args.image_info_topic, 'depth_info': args.depth_info_topic, 'extrinsics': args.extrinsics_topic}
            result = extract_mcap(args.bag, args.output, topics, args.session, args.depth_scale_m, args.sync_tolerance_ms, args.start_sec, args.duration_sec, args.every)
        elif args.command == 'annotate':
            root, index = load_index(args.index)
            matches = [r for r in index['samples'] if r['sample'] == args.sample]
            if len(matches) != 1: raise ValueError('sample_not_unique_or_missing')
            frame = read_sample(root, matches[0])
            target = Path(args.output); target.mkdir(parents=True, exist_ok=False)
            if args.prepare:
                image = frame.image if frame.image.ndim == 2 else cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR)
                if not cv2.imwrite(str(target / 'original.png'), image): raise OSError('cannot export original PNG')
                dump(target / 'source.json', {'source_index_sha256': sha256(args.index), 'sample': matches[0], 'image_sha256': sha256(target / 'original.png')})
                print(json.dumps({'status': 'prepared_not_annotated', 'image': str(target / 'original.png')}, ensure_ascii=False)); return 0
            if not args.annotation: raise ValueError('--annotation or --prepare required')
            annotation = json.loads(Path(args.annotation).read_text())
            if annotation.get('annotation_type') == 'manual':
                if not args.approve or not args.author: raise ValueError('legacy manual format requires explicit --approve --author')
                if annotation.get('coordinate_system') != 'original_image_pixels': raise ValueError('legacy ROI pixels cannot be silently converted')
                if sha256(annotation['input_path']) != annotation['input_sha256']: raise ValueError('manual input image hash mismatch')
                read = cv2.imread(annotation['input_path'], cv2.IMREAD_GRAYSCALE if frame.image.ndim == 2 else cv2.IMREAD_COLOR)
                if frame.image.ndim == 3: read = cv2.cvtColor(read, cv2.COLOR_BGR2RGB)
                if not np.array_equal(read, frame.image): raise ValueError('manual annotation belongs to different original image')
                annotation = {'status': 'approved', 'author': args.author, 'origin': 'human',
                              'left_xy': [[p['x'], p['y']] for p in annotation['left_boundary_manual_points']],
                              'right_xy': [[p['x'], p['y']] for p in annotation['right_boundary_manual_points']],
                              'legacy_annotation_sha256': sha256(args.annotation)}
            mask, _ = annotation_mask(annotation, frame.image.shape)
            matches[0]['annotation'] = dict(annotation, interpolation='linear_in_original_y', annotation_sha256=sha256(args.annotation))
            # Absolute external data references are intentionally avoided; copy ONLY approved sample into new index.
            record = save_sample(target, frame, Path(args.sample).stem, matches[0]['session'], matches[0]['split'], matches[0]['annotation'])
            dump(target / 'index.json', {'schema': 'd405_dataset_v1', 'samples': [record], 'sources': index.get('sources', [])})
            display = cv2.cvtColor(frame.image, cv2.COLOR_GRAY2BGR) if frame.image.ndim == 2 else cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR)
            display[mask.astype(bool)] = display[mask.astype(bool)] // 2 + [0, 80, 0]
            cv2.imwrite(str(target / 'annotation_overlay.png'), display)
            result = {'status': 'annotation_verified', 'sample': record}
        elif args.command == 'benchmark':
            from .evaluation import Evaluator
            if Path(args.output).exists(): raise FileExistsError(args.output)
            if args.iterations < 1: raise ValueError('positive_iterations_required')
            root, index = load_index(args.index); frame = read_sample(root, index['samples'][0]); config = load_config(args.config)
            pipe, ev = Pipeline(config), Evaluator()
            try:
                for i in range(args.iterations):
                    f = replace(frame, sequence=i, image_stamp_ns=frame.image_stamp_ns + i * 100_000_000, depth_stamp_ns=None if frame.depth_stamp_ns is None else frame.depth_stamp_ns + i * 100_000_000)
                    r = pipe.process(f); ev.add(f, r, None, None)
                result = dict(ev.report(), benchmark='repeated_fixed_input_compute_only; synthetic advancing time; NOT live sensor FPS', actual_backends=r['actual_backends'], sample_sha256=index['samples'][0]['sha256'])
                dump(args.output, result)
            finally: pipe.close()
        elif args.command == 'rerun':
            import os, subprocess
            python = os.environ.get('FIELD_RRD_PYTHON')
            if not python or not Path(python).is_file(): raise RuntimeError('source existing system env; FIELD_RRD_PYTHON required')
            # Existing Rerun 0.36 requires NumPy2; run its adapter in its OWN environment.
            # No package installation or NumPy upgrade in production/core/training environments.
            subprocess.run([python, str(Path(__file__).with_name('rerun_adapter.py')), '--index', args.index,
                            '--results', args.results, '--output', args.output], check=True)
            result = {'rrd': args.output, 'sha256': sha256(args.output), 'field_accuracy_validated': False}
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)); return 0
    except Exception as exc:
        import sys
        print(json.dumps({'status': 'failed', 'reason': type(exc).__name__ + ': ' + str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == '__main__': raise SystemExit(main())
