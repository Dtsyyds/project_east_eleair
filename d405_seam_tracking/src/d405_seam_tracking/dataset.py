"""Lossless indexed samples, explicit approved labels and bounded typed MCAP extraction."""
from dataclasses import asdict
import json
from pathlib import Path
import cv2
import numpy as np
from .contracts import Calibration, Frame, dump, sha256


def safe_path(root, relative):
    root = Path(root).resolve()
    target = (root / relative).resolve()
    if not target.is_relative_to(root): raise ValueError('dataset_path_escape')
    return target


def save_sample(root, frame, name, session, split='unassigned', annotation=None):
    root = Path(root)
    path = root / (name + '.npz')
    if path.exists(): raise FileExistsError(path)
    # No object arrays; allow_pickle=False on read. RGB and uint16 depth preserved verbatim.
    metadata = frame.metadata()
    metadata['source'] = dict(metadata['source'], session=session)
    np.savez_compressed(path, image=frame.image, depth=np.array([], dtype=np.uint16) if frame.depth is None else frame.depth,
                        raw_valid=np.array([], dtype=bool) if frame.raw_valid is None else frame.raw_valid,
                        metadata=np.array(json.dumps(metadata, allow_nan=False)))
    return {'sample': path.name, 'sha256': sha256(path), 'session': session, 'split': split, 'annotation': annotation}


def read_sample(root, record, verify=True):
    p = safe_path(root, record['sample'])
    if verify and sha256(p) != record['sha256']: raise ValueError('sample_hash_mismatch')
    with np.load(p, allow_pickle=False) as n:
        m = json.loads(str(n['metadata']))
        c = lambda k: Calibration(**m[k]) if m.get(k) else None
        frame = Frame(n['image'], n['depth'] if n['depth'].size else None, n['raw_valid'] if n['raw_valid'].size else None,
                      int(m['image_stamp_ns']), None if m['depth_stamp_ns'] is None else int(m['depth_stamp_ns']), m['sequence'],
                      c('image_calibration'), c('depth_calibration'), m['depth_to_image'], m['depth_scale_m'], m['source'],
                      None if m.get('receive_stamp_ns') is None else int(m['receive_stamp_ns']))
    return frame.validate()


def load_index(path):
    p = Path(path)
    index = json.loads(p.read_text())
    if index['schema'] != 'd405_dataset_v1': raise ValueError('unsupported_dataset_schema')
    return p.parent, index


def split_check(records):
    assignment = {}
    for r in records:
        if r['split'] not in ('train', 'val', 'test', 'unassigned'): raise ValueError('invalid_split')
        prior = assignment.setdefault(r['session'], r['split'])
        if prior != r['split']: raise ValueError('session_leaks_across_splits')
    return {'session_count': len(assignment), 'independent_generalization_sufficient': len(assignment) >= 3}


def annotation_mask(annotation, shape, allow_synthetic=False):
    if annotation.get('status') != 'approved' or not annotation.get('author') or annotation.get('origin') not in (('human', 'synthetic') if allow_synthetic else ('human',)):
        raise ValueError('human_approved_annotation_required')
    mask = np.zeros(shape[:2], np.uint8)
    if annotation.get('no_seam') is True: return mask, []
    left, right = np.asarray(annotation['left_xy'], float), np.asarray(annotation['right_xy'], float)
    h, w = shape[:2]
    for a in (left, right):
        if a.ndim != 2 or a.shape[1] != 2 or len(a) < 2 or not np.isfinite(a).all() or np.any(np.diff(a[:, 1]) <= 0):
            raise ValueError('boundary_points_must_have_increasing_y')
        if (a < 0).any() or (a[:, 0] >= w).any() or (a[:, 1] >= h).any(): raise ValueError('annotation_outside_image')
    first, last = int(np.ceil(max(left[0, 1], right[0, 1]))), int(np.floor(min(left[-1, 1], right[-1, 1])))
    if last < first: raise ValueError('no_common_boundary_y')
    ys = np.arange(first, last + 1)
    l, r = np.interp(ys, left[:, 1], left[:, 0]), np.interp(ys, right[:, 1], right[:, 0])
    if (l >= r).any(): raise ValueError('crossed_boundaries')
    for y, x0, x1 in zip(ys, l, r): mask[y, int(np.ceil(x0)):int(np.floor(x1)) + 1] = 1
    return mask, np.column_stack((ys, l, r)).tolist()


def label_valid_region(shape, rows):
    valid = np.ones(shape[:2], dtype=bool)
    if rows:
        valid[:] = False
        valid[int(rows[0][0]):int(rows[-1][0]) + 1] = True
    return valid


def from_camera_info(message, source):
    return Calibration(message.width, message.height, list(message.k), list(message.d), message.distortion_model,
                       message.header.frame_id, source).validate()


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def image_array(message):
    enc = message.encoding.lower()
    contract = {'mono8': (np.uint8, 1), 'rgb8': (np.uint8, 3), 'bgr8': (np.uint8, 3), '16uc1': (np.uint16, 1), '32fc1': (np.float32, 1)}
    if enc not in contract: raise ValueError('unsupported_image_encoding:' + enc)
    dtype, channels = contract[enc]
    dtype = np.dtype(dtype).newbyteorder('>' if message.is_bigendian else '<')
    size = dtype.itemsize * channels * message.width
    if message.step < size or len(message.data) < message.step * message.height: raise ValueError('image_stride_or_payload_invalid')
    # Borrow the message array through NumPy's retained base object. Copy only endian/BGR conversion.
    a = np.ndarray((message.height, message.width, channels), dtype=dtype, buffer=memoryview(message.data),
                   strides=(message.step, dtype.itemsize * channels, dtype.itemsize)).astype(dtype.newbyteorder('='), copy=False)
    if channels == 1: return a[:, :, 0]
    return cv2.cvtColor(a, cv2.COLOR_BGR2RGB) if enc == 'bgr8' else a


class Synchronizer:
    """At most capacity images on each side; each source consumed once. No restamping."""
    def __init__(self, tolerance_ns, capacity=6):
        self.tolerance_ns, self.capacity = tolerance_ns, capacity
        self.images, self.depths = [], []
        self.counts = {'image_received': 0, 'depth_received': 0, 'matched': 0, 'expired_image': 0, 'expired_depth': 0, 'unmatched_image': 0, 'unmatched_depth': 0}
        self.last = {'image': None, 'depth': None}
        self.reset_count = 0

    def add(self, kind, message):
        stamp = stamp_ns(message)
        if stamp <= 0: raise ValueError('invalid_source_stamp')
        if self.last[kind] is not None and stamp <= self.last[kind]:
            self.counts['expired_image'] += len(self.images); self.counts['expired_depth'] += len(self.depths)
            self.images.clear(); self.depths.clear(); self.reset_count += 1
            self.last = {'image': None, 'depth': None}
        self.last[kind] = stamp
        queue = self.images if kind == 'image' else self.depths
        self.counts[kind + '_received'] += 1
        queue.append(message)
        if len(queue) > self.capacity:
            queue.pop(0); self.counts['expired_' + kind] += 1
        pairs = []
        while self.images and self.depths:
            i, d = self.images[0], self.depths[0]
            delta = stamp_ns(d) - stamp_ns(i)
            if abs(delta) <= self.tolerance_ns:
                pairs.append((self.images.pop(0), self.depths.pop(0))); self.counts['matched'] += 1
            elif delta < 0:
                self.depths.pop(0); self.counts['expired_depth'] += 1
            else:
                self.images.pop(0); self.counts['expired_image'] += 1
        return pairs

    def finish(self):
        self.counts['unmatched_image'] = len(self.images); self.counts['unmatched_depth'] = len(self.depths)
        return dict(self.counts, source_time_resets=self.reset_count)


def extract_mcap(paths, output, topics, session, depth_scale_m, tolerance_ms=12., start_sec=0., duration_sec=None, every=1):
    """Explicit bags only, actual registered ROS types/CDR. Stream bounded heap across bags."""
    import heapq
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    if every < 1 or depth_scale_m is None or depth_scale_m <= 0: raise ValueError('explicit_depth_scale_and_positive_sampling_required')
    root = Path(output); root.mkdir(parents=True, exist_ok=False)
    readers, types, heap, calibration, extrinsics = [], [], [], {}, None
    source_hashes = []
    for path in paths:
        p = Path(path).resolve()
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(p), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
        mapping = {t.name: t.type for t in reader.get_all_topics_and_types()}
        for role in ('image', 'depth', 'image_info', 'depth_info', 'extrinsics'):
            if topics[role] not in mapping: raise ValueError('required_topic_missing:' + topics[role])
        expected = {'image': 'sensor_msgs/msg/Image', 'depth': 'sensor_msgs/msg/Image', 'image_info': 'sensor_msgs/msg/CameraInfo', 'depth_info': 'sensor_msgs/msg/CameraInfo', 'extrinsics': 'realsense2_camera_msgs/msg/Extrinsics'}
        for role, msgtype in expected.items():
            if mapping[topics[role]] != msgtype: raise ValueError('unexpected_ros_type:' + topics[role])
        selected = {topics[r]: get_message(t) for r, t in expected.items()}
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(selected)))
        readers.append(reader); types.append(selected)
        files = [p] if p.is_file() else sorted(p.rglob('*.mcap'))
        if not p.is_file() and (p / 'metadata.yaml').exists(): files.append(p / 'metadata.yaml')
        source_hashes += [{'path': str(f), 'sha256': sha256(f)} for f in files]
        if reader.has_next():
            topic, raw, ts = reader.read_next(); heapq.heappush(heap, (ts, len(readers) - 1, topic, raw))
    sync = Synchronizer(int(tolerance_ms * 1e6), 8)
    records, sequence, first = [], 0, None
    skipped_calibration = 0
    while heap:
        ts, bag, topic, raw = heapq.heappop(heap)
        if first is None: first = ts
        relative = (ts - first) / 1e9
        message = deserialize_message(raw, types[bag][topic])
        if topic == topics['image_info']: calibration['image'] = from_camera_info(message, topic)
        elif topic == topics['depth_info']: calibration['depth'] = from_camera_info(message, topic)
        elif topic == topics['extrinsics']:
            extrinsics = {'rotation': np.array(message.rotation).reshape(3, 3, order='F').ravel().tolist(), 'translation_m': list(message.translation), 'source': topic}
        elif relative >= start_sec and (duration_sec is None or relative <= start_sec + duration_sec):
            kind = 'image' if topic == topics['image'] else 'depth'
            for image, depth in sync.add(kind, message):
                sequence += 1
                if sequence % every: continue
                if len(calibration) != 2 or extrinsics is None:
                    skipped_calibration += 1; continue
                e = dict(extrinsics, from_frame=calibration['depth'].frame_id, to_frame=calibration['image'].frame_id)
                a, d = image_array(image), image_array(depth)
                frame = Frame(a, d, np.isfinite(d) & (d > 0), stamp_ns(image), stamp_ns(depth), sequence,
                              calibration['image'], calibration['depth'], e, depth_scale_m,
                              {'bags': [str(Path(p).resolve()) for p in paths], 'topics': topics, 'bag_receive_stamp_ns': str(ts), 'session': session}).validate()
                records.append(save_sample(root, frame, f'frame_{sequence:08d}', session))
        if readers[bag].has_next():
            topic, raw, next_ts = readers[bag].read_next(); heapq.heappush(heap, (next_ts, bag, topic, raw))
    if not records: raise ValueError('no_calibrated_pairs_extracted; check time range/CameraInfo/extrinsics')
    index = {'schema': 'd405_dataset_v1', 'samples': records, 'sources': source_hashes, 'topics': topics,
             'sync_counts': sync.finish(), 'skipped_missing_calibration': skipped_calibration, 'sample_every': every, 'calibration_status': 'factory_not_rig_calibrated'}
    dump(root / 'index.json', index)
    return index
