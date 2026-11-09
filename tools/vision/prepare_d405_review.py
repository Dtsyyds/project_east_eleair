#!/usr/bin/env python3
"""Prepare a read-only sample review, or import its explicit human annotations.

Run after sourcing deploy/env.sh. Outputs must be new directories. Source samples
remain untouched; exported labels are bound to the exact index and raw SHA256.
"""
import argparse
import json
from pathlib import Path
import shutil
import hashlib
import threading
from datetime import datetime, timezone
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import cv2
import numpy as np

from d405_seam_tracking.contracts import dump, sha256
from d405_seam_tracking.dataset import annotation_mask, load_index, read_sample, safe_path


def validate_label(label, frame):
    if not isinstance(label, dict) or label.get('origin') != 'human':
        raise ValueError('explicit_human_annotation_required')
    if label.get('status') != 'approved' or not str(label.get('author', '')).strip():
        raise ValueError('annotation_author_required')
    if label.get('no_seam') is not True:
        for side in ('left_xy', 'right_xy'):
            points = np.asarray(label.get(side, []), dtype=float)
            if points.ndim != 2 or points.shape[1] != 2 or len(points) < 6:
                raise ValueError('six_points_per_boundary_required')
            if not np.isfinite(points).all() or np.any(np.diff(points[:, 1]) <= 0):
                raise ValueError('boundary_y_must_strictly_increase')
        left, right = np.asarray(label['left_xy']), np.asarray(label['right_xy'])
        lo, hi = max(left[0, 1], right[0, 1]), min(left[-1, 1], right[-1, 1])
        if hi - lo < 5:
            raise ValueError('insufficient_common_visible_region')
        ys = np.unique(np.r_[lo, hi, left[:, 1], right[:, 1]])
        ys = ys[(ys >= lo) & (ys <= hi)]
        if np.any(np.interp(ys, left[:, 1], left[:, 0]) >= np.interp(ys, right[:, 1], right[:, 0])):
            raise ValueError('crossing_boundaries')
    annotation_mask(label, frame.image.shape)
    return label


def import_labels(index_path, labels_path, output):
    root, index = load_index(index_path)
    labels = json.loads(Path(labels_path).read_text())
    if labels.get('schema') != 'd405_review_labels_v1' or labels.get('source_index_sha256') != sha256(index_path):
        raise ValueError('labels_belong_to_different_dataset')
    if labels.get('target') != 'visible_weld_boundaries':
        raise ValueError('unsupported_annotation_target')
    records = {r['sample']: r for r in index['samples']}
    if len(records) != len(index['samples']):
        raise ValueError('duplicate_source_sample')
    checked, seen = [], set()
    for item in labels['samples']:
        sample = item['sample']
        if sample not in records or sample in seen:
            raise ValueError('unknown_or_duplicate_label_sample')
        seen.add(sample)
        rec = records[sample]
        if item['raw_sha256'] != rec['sha256']:
            raise ValueError('label_sample_hash_mismatch')
        frame = read_sample(root, rec)
        checked.append((rec, validate_label(item['annotation'], frame)))
    if not checked:
        raise ValueError('no_human_labels')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    imported = []
    for i, (rec, label) in enumerate(checked, 1):
        name = f'sample_{i:03d}.npz'
        shutil.copy2(safe_path(root, rec['sample']), output / name)
        if sha256(output / name) != rec['sha256']:
            raise ValueError('copied_sample_hash_mismatch')
        imported.append(dict(rec, sample=name, annotation=label, split='unassigned'))
    dump(output / 'index.json', dict(index, samples=imported,
         annotation_source_sha256=sha256(labels_path), source_index_sha256=sha256(index_path)))
    dump(output / 'import_report.json', {'source_count': len(records), 'labelled_count': len(checked),
         'source_unchanged': True, 'split': 'unassigned',
         'note': 'Do not randomly split snapshots of the same physical scene into train/test.'})
    return {'output': str(output), 'labelled': len(checked)}


class ReviewLabelStore:
    """Keep immutable received revisions; never modify raw data or browser labels."""
    def __init__(self, index_path, output):
        root, index = load_index(index_path)
        self.index_hash = sha256(index_path)
        self.records = {r['sample']: r for r in index['samples']}
        self.frames = {name: read_sample(root, r) for name, r in self.records.items()}
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()

    def receive(self, value):
        if value.get('schema') != 'd405_review_labels_v1' or value.get('source_index_sha256') != self.index_hash:
            raise ValueError('labels_belong_to_different_dataset')
        if value.get('target') != 'visible_weld_boundaries':
            raise ValueError('unsupported_annotation_target')
        samples = value.get('samples')
        if not isinstance(samples, list) or not 0 < len(samples) <= len(self.records):
            raise ValueError('invalid_annotation_count')
        seen = set()
        for item in samples:
            name = item['sample']
            if name not in self.records or name in seen:
                raise ValueError('unknown_or_duplicate_label_sample')
            seen.add(name)
            if item['raw_sha256'] != self.records[name]['sha256']:
                raise ValueError('label_sample_hash_mismatch')
            validate_label(item['annotation'], self.frames[name])
        body = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + '\n').encode()
        digest = hashlib.sha256(body).hexdigest()
        result = {'count': len(samples), 'total': len(self.records), 'sha256': digest,
                  'file': 'labels_' + digest + '.json', 'received_at': datetime.now(timezone.utc).isoformat()}
        with self.lock:
            path = self.output / result['file']
            if not path.exists():
                with path.open('xb') as stream:
                    stream.write(body)
            dump(self.output / 'latest_received.json', result)
        return result


def serve(index_path, directory, port):
    directory = Path(directory).resolve()
    provenance = json.loads((directory / 'provenance.json').read_text())
    if provenance['source_index_sha256'] != sha256(index_path):
        raise ValueError('review_page_belongs_to_different_dataset')
    store = ReviewLabelStore(index_path, directory.parent / 'received_labels')

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(directory), **kwargs)

        def end_headers(self):
            self.send_header('Cache-Control', 'no-store')
            super().end_headers()

        def list_directory(self, path):
            self.send_error(403)
            return None

        def reply(self, status, value):
            body = json.dumps(value, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if self.path != '/api/labels':
                self.reply(404, {'error': 'unknown_endpoint'})
                return
            origin = self.headers.get('Origin')
            if origin and urlsplit(origin).netloc != self.headers.get('Host'):
                self.reply(403, {'error': 'origin_mismatch'})
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 2 * 1024 * 1024:
                    raise ValueError('invalid_request_size')
                if self.headers.get('Content-Type', '').split(';')[0].strip() != 'application/json':
                    raise ValueError('json_required')
                value = json.loads(self.rfile.read(length))
                if not isinstance(value, dict):
                    raise ValueError('invalid_label_payload')
                result = store.receive(value)
            except (ValueError, KeyError, TypeError, IndexError, AttributeError) as error:
                self.reply(400, {'error': str(error)})
                return
            self.reply(200, result)

    server = ThreadingHTTPServer(('0.0.0.0', port), Handler)
    print(f'Review server listening on {port}; label revisions: {store.output}', flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()


def prepare(index_path, output):
    root, index = load_index(index_path)
    manifest = json.loads((root / 'samples/result_manifest.json').read_text())
    if manifest.get('complete') is not True:
        raise ValueError('stop_vision_and_seal_writer_first')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    assets = output / 'assets'
    assets.mkdir()
    rows = []
    for number, rec in enumerate(index['samples'], 1):
        frame = read_sample(root, rec)
        directory = safe_path(root, rec['sample']).parent
        metadata = json.loads((directory / 'metadata.json').read_text())
        if metadata.get('complete') is not True:
            raise ValueError('incomplete_sample')
        for file in metadata['files']:
            if sha256(safe_path(directory, file['path'])) != file['sha256']:
                raise ValueError('event_file_hash_mismatch:' + file['path'])
        result = json.loads((directory / 'result.json').read_text())
        if int(result['source']['image_stamp_ns']) != frame.image_stamp_ns:
            raise ValueError('result_frame_mismatch')
        image = frame.image if frame.image.ndim == 2 else cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR)
        filename = f'assets/{number:02d}.png'
        if not cv2.imwrite(str(output / filename), image):
            raise OSError('image_export_failed')
        depth = result.get('depth', {})
        rows.append({'id': number, 'sample': rec['sample'], 'raw_sha256': rec['sha256'],
                     'image': filename, 'image_sha256': sha256(output / filename),
                     'width': frame.image.shape[1], 'height': frame.image.shape[0],
                     'state': result['state'], 'points': result['points_2d_y_left_right'],
                     'visual_reasons': result['visual'].get('reasons', []),
                     'visual_coverage': result['visual'].get('coverage', 0),
                     'depth_reasons': depth.get('reasons', []),
                     'depth_valid_ratio': depth.get('valid_ratio'),
                     'depth_threshold_mm': depth.get('height_threshold_m', 0) * 1000,
                     'metric_valid': result['metric_valid']})
    data = {'schema': 'd405_review_v1', 'source_index_sha256': sha256(index_path),
            'source_session': root.name, 'samples': rows}
    dump(output / 'review.json', data)
    template = Path(__file__).with_name('d405_review.html').read_text()
    payload = json.dumps(data, ensure_ascii=False, allow_nan=False).replace('<', '\\u003c')
    (output / 'index.html').write_text(template.replace('/*REVIEW_DATA*/null', payload))
    dump(output / 'provenance.json', {'source_index': str(Path(index_path).resolve()),
         'source_index_sha256': sha256(index_path), 'samples_verified': len(rows),
         'source_manifest_sha256': sha256(root / 'samples/result_manifest.json'),
         'original_annotations_modified': False, 'automatic_ground_truth': False})
    return {'output': str(output), 'samples': len(rows), 'status': 'ready_for_human_annotation'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--index', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--import-labels', help='Import explicitly saved human labels instead of preparing the review')
    p.add_argument('--serve', action='store_true', help='Serve an existing review with validated label synchronization')
    p.add_argument('--port', type=int, default=8081)
    args = p.parse_args()
    if args.serve:
        if args.import_labels:
            p.error('--serve cannot be combined with --import-labels')
        serve(args.index, args.output, args.port)
        return
    result = (import_labels(args.index, args.import_labels, args.output) if args.import_labels
              else prepare(args.index, args.output))
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
