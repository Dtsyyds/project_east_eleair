"""Standalone bridge into the existing matched Rerun environment (NumPy2).

No import of ROS/training/core package. Used only by d405-seam rerun.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import rerun as rr


def main():
    p = argparse.ArgumentParser(); p.add_argument('--index', required=True); p.add_argument('--results', required=True); p.add_argument('--output', required=True)
    a = p.parse_args()
    output = Path(a.output); partial = output.with_name(output.name + '.partial')
    if output.exists() or partial.exists(): raise FileExistsError(output)
    index_path = Path(a.index).resolve(); root = index_path.parent
    index = json.loads(index_path.read_text())
    rr.init('d405_seam_experiment'); rr.save(partial)
    sources = []
    try:
        with Path(a.results).open() as stream:
            for record in index['samples']:
                sample = (root / record['sample']).resolve()
                if not sample.is_relative_to(root): raise ValueError('source_path_escape')
                actual = hashlib.sha256(sample.read_bytes()).hexdigest()
                if actual != record['sha256']: raise ValueError('source_hash_mismatch')
                with np.load(sample, allow_pickle=False) as n:
                    metadata = json.loads(str(n['metadata'])); image = n['image']
                    depth = n['depth'] if 'depth' in n else None
                    valid = n['raw_valid'] if 'raw_valid' in n else None
                result = json.loads(next(stream))
                if result['source']['image_stamp_ns'] != metadata['image_stamp_ns'] or result['source']['sequence'] != metadata['sequence']:
                    raise ValueError('result_frame_mismatch')
                rr.set_time('source', timestamp=int(metadata['image_stamp_ns']) / 1e9)
                rr.log('camera/image', rr.Image(image)); rr.log('result', rr.TextDocument(json.dumps(result, ensure_ascii=False)))
                if depth is not None and depth.size:
                    scale = metadata.get('depth_scale_m')
                    if scale is None or scale <= 0: raise ValueError('explicit_depth_scale_required')
                    # Original depth coordinate system, not resized/aligned to the image.
                    rr.log('depth/original', rr.DepthImage(depth, meter=1./scale))
                    rr.log('depth/raw_valid', rr.Image(valid.astype(np.uint8) * 255))
                points = np.asarray(result['points_2d_y_left_right'])
                color = [220, 0, 220] if result['predicted_only'] else ([255, 0, 0] if result['state'] == 'CONFLICT_2D' else ([0, 200, 0] if result['seam_identity_confirmed'] else [255, 170, 0]))
                for name, x in [('left', points[:, 1] if len(points) else []), ('right', points[:, 2] if len(points) else []), ('center', points[:, 1:].mean(1) if len(points) else [])]:
                    if len(points): rr.log('camera/image/' + name, rr.Points2D(np.column_stack((x, points[:, 0])), colors=color))
                    else: rr.log('camera/image/' + name, rr.Clear(recursive=True))
                sources.append({'path': str(sample), 'sha256': actual})
            if stream.read().strip(): raise ValueError('extra_result_frames')
    finally: rr.disconnect()
    if output.exists(): raise FileExistsError(output)
    partial.rename(output)
    manifest = {'schema': 'd405_rrd_v1', 'complete': True, 'sources': sources,
                'source_index_sha256': hashlib.sha256(index_path.read_bytes()).hexdigest(),
                'results_sha256': hashlib.sha256(Path(a.results).read_bytes()).hexdigest(),
                'output_sha256': hashlib.sha256(output.read_bytes()).hexdigest(), 'accuracy_validated': False}
    output.with_name(output.name + '.manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__': main()
