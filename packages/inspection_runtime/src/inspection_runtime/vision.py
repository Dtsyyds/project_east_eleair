"""Owned, passive D405 experiment process. No camera owner or actuator commands."""
from __future__ import annotations
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid

from .paths import config_path, project_root

SELECTION_ACTIONS = ('vision.select_candidate', 'vision.clear_selection')
ACTIONS = ('vision.start', 'vision.capture_start', 'vision.stop', 'vision.save_sample', 'vision.status') + SELECTION_ACTIONS
ACTIVE = ('starting', 'running', 'stopping')
INPUTS = ('/d405/camera/color/image_raw', '/d405/camera/depth/image_rect_raw')


def runtime_command(config, capture_only=False):
    """Check local deployment artifacts before creating an experiment or process."""
    import yaml
    if capture_only:
        return Path(config_path('d405/capture.yaml')), [sys.executable, str(project_root() / 'tools/vision/run_d405_capture.py')]
    vision = config.get('vision', {})
    python = vision.get('python')
    if python:
        interpreter = project_root() / python
        if not interpreter.is_file() or not os.access(interpreter, os.X_OK):
            raise RuntimeError('配置的巡迹 Python 环境不可用：' + str(interpreter))
        command = [str(interpreter), str(project_root() / 'tools/vision/run_d405_node.py')]
    else:
        command = ['ros2', 'run', 'd405_seam_ros', 'd405_seam_node']
    cfg = Path(config_path(vision.get('algorithm_config', 'd405/experimental.yaml')))
    algorithm = yaml.safe_load(cfg.read_text())
    if algorithm.get('algorithm') == 'round1_a':
        checkpoint = project_root() / algorithm['checkpoint']
        expected = algorithm['checkpoint_sha256']
        if not checkpoint.is_file():
            raise RuntimeError('巡迹模型权重缺失：' + str(checkpoint) + '；需要恢复原文件，SHA256=' + expected)
        if hashlib.sha256(checkpoint.read_bytes()).hexdigest() != expected:
            raise RuntimeError('巡迹模型权重校验失败：' + str(checkpoint) + '；期望 SHA256=' + expected)
    return cfg, command


def write_json(path, value):
    temp = path.with_suffix(path.suffix + '.partial')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    temp.replace(path)


def tracking_payload(result, preview_width):
    """Map original Color pixels to the bridge's exact resized preview convention."""
    source = result['source']
    if source['source']['stream'] != 'color':
        raise ValueError('巡迹图像不是当前 D405 彩色预览来源')
    width, height = source['image_width'], source['image_height']
    if not (isinstance(width, int) and isinstance(height, int) and width > 0 and height > 0):
        raise ValueError('巡迹源尺寸无效')
    scale = min(1., preview_width / width)
    w, h = max(1, int(width * scale)), max(1, int(height * scale))
    def point(x, y):
        if not math.isfinite(x) or not math.isfinite(y): raise ValueError('巡迹坐标无效')
        return [(x + .5) * w / width - .5, (y + .5) * h / height - .5]
    rows = result['points_2d_y_left_right'][:4000] if result['valid_2d'] else []
    center_rows = result.get('points_2d_y_center', [])[:4000] if result.get('state') == 'CENTER_2D' and result['valid_2d'] else []
    ref = {'camera_id': 'depth_camera', 'frame_id': source['source']['image_frame_id'],
           'stamp_ns': source['image_stamp_ns']}
    return {'status': result['state'], 'source': 'D405 Color' if result['mode'] == 'vision_only' else 'D405 Color + Depth', 'frame_ref': ref,
            'stamp_ns': ref['stamp_ns'], 'coordinate_space': 'image_pixels',
            'image_width': w, 'image_height': h, 'mode': result['mode'],
            'vision_2d_valid': result['valid_2d'], 'geometry_3d_valid': result['metric_valid'],
            'identity_confirmed': result['seam_identity_confirmed'], 'predicted': result['predicted_only'],
            'backend_version': result.get('actual_backends', {}).get('visual'),
            'model_version': result.get('model_sha256'),
            'reason': '；'.join(result['reasons']) or '实验巡迹结果；焊缝精度未验收',
            'left_boundary': [point(left, y) for y, left, right in rows],
            'right_boundary': [point(right, y) for y, left, right in rows],
            'centerline': [point(x, y) for y, x in center_rows] if center_rows else [point((left + right) / 2, y) for y, left, right in rows],
            'session_id': source['source'].get('session'),
            'candidate_set_id': result.get('candidate_set_id'),
            'candidates': [dict(candidate_id=c['candidate_id'], rank=c['rank'], score=c['score'],
                                coverage=c['coverage'], confidence=c['confidence'],
                                centerline=[point(x, y) for y, x in c['points_2d_y_center']])
                           for c in result.get('candidates', [])[:3]],
            'selection': result.get('selection'),
            'diagnostics': {'confidence': result['confidence'], 'body_coordinate_valid': False,
                            'processing_ms': result['timing_ms']['total']}}


class VisionController:
    def __init__(self, config, storage, live_topics, service_call, popen=subprocess.Popen):
        self.config, self.storage, self.live_topics = config, storage, live_topics
        self.service_call, self.popen = service_call, popen
        self.lock = threading.RLock()
        self.process = None
        self.log = None
        self.session = None
        self.last_diagnostic = 0.
        self.state = {'status': 'idle', 'message': '未启动视觉巡迹', 'sample': None}

    def snapshot(self):
        with self.lock:
            if self.process and self.process.poll() is not None and self.state['status'] != 'stopped':
                self.state.update(status='failed', message='巡迹进程已退出，请查看本次实验日志')
            value = copy.deepcopy(self.state)
            value['diagnostics_fresh'] = self.last_diagnostic > 0 and time.monotonic() - self.last_diagnostic < 3.
            return value

    def diagnostic(self, data):
        with self.lock:
            # Ignore stale or independent publishers, including a previous experiment.
            if data.get('session_id') != self.state.get('session_id') or self.state['status'] not in ('starting', 'running'):
                return
            self.last_diagnostic = time.monotonic()
            self.state['diagnostics'] = data
            label = '原始图片采集 · Color + Depth' if self.state.get('capture_only') else self.config.get('vision', {}).get('label', 'Color + Depth')
            self.state.update(status='running', algorithm_label=label, message=label + '运行中 · 不控制机器人')

    def check_storage(self):
        s = self.storage()
        if not s['mounted'] or s['free_bytes'] < self.config['minimum_free_bytes']:
            raise RuntimeError('实验数据盘未挂载或空间不足')

    def check_inputs(self):
        live, now = self.live_topics(), time.monotonic()
        missing = [t for t in INPUTS if now - live.get(t, -math.inf) >= 3.]
        if missing: raise RuntimeError('D405 数据未就绪或已过期：' + '、'.join(missing))

    def select_candidate(self, parameters, clear=False):
        fields = {'session_id'} if clear else {'session_id', 'candidate_set_id', 'candidate_id'}
        if set(parameters) != fields or any(not isinstance(v, str) or not 1 <= len(v) <= 128 for v in parameters.values()):
            raise ValueError('候选选择参数无效，请从当前画面重新选择')
        with self.lock:
            state = self.snapshot()
            if state['status'] != 'running' or not state['diagnostics_fresh']:
                raise RuntimeError('巡迹未运行或诊断过期')
            if parameters['session_id'] != state.get('session_id'):
                raise RuntimeError('巡迹会话已变化，请从当前画面重新选择')
            if not state.get('diagnostics', {}).get('candidate_selection_supported'):
                raise RuntimeError('当前节点不支持候选选择，请使用更新后的 candidate 模式')
        request = dict(parameters, request_id=uuid.uuid4().hex, action='clear' if clear else 'select')
        reply = self.service_call('select_candidate', request)
        with self.lock:
            self.state['selection'] = reply['selection']
        return reply

    def start(self, capture_only=False):
        with self.lock:
            if self.process and self.process.poll() is None:
                raise RuntimeError('视觉巡迹已经启动')
            self.check_storage(); self.check_inputs()
            if self.service_call('probe').get('available'):
                raise RuntimeError('已有独立巡迹节点，请先在其启动终端停止，避免重复进程')
            try:
                cfg, command = runtime_command(self.config, capture_only=capture_only)
            except (OSError, RuntimeError) as error:
                self.state.update(status='failed', message=str(error))
                raise
            experiment = 'VISION-' + time.strftime('%Y%m%d_%H%M%S') + '-' + uuid.uuid4().hex[:8]
            self.session = Path(self.config['data_root']) / 'vision_experiments' / experiment
            self.session.mkdir(parents=True, exist_ok=False)
            frozen = self.session / 'config.yaml'; frozen.write_bytes(cfg.read_bytes())
            write_json(self.session / 'session.json', {'schema': 'vision_experiment_v1', 'session_id': experiment,
                       'config_sha256': hashlib.sha256(frozen.read_bytes()).hexdigest(), 'stream': 'color',
                       'source': 'live_ros', 'annotation_status': 'unlabelled', 'hardware_validated': False})
            if self.log: self.log.close()
            self.log = (self.session / 'node.log').open('wb')
            self.state = {'status': 'starting', 'session_id': experiment, 'directory': str(self.session),
                          'message': '正在启动，等待节点诊断', 'sample': None, 'saved_count': 0, 'capture_only': capture_only}
            self.last_diagnostic = 0.
            try:
                self.process = self.popen(command + [
                    '--config', str(frozen), '--stream', 'color', '--depth-scale-m', '0.001',
                    '--runtime-mode', 'capture' if capture_only else self.config.get('vision', {}).get('runtime_mode', 'a'),
                    '--session-id', experiment, '--events', str(self.session / 'samples')], stdout=self.log, stderr=subprocess.STDOUT,
                    start_new_session=True)
            except Exception:
                self.log.close(); self.state.update(status='failed', message='视觉巡迹启动失败'); raise
        deadline = time.monotonic() + 12.
        while time.monotonic() < deadline:
            value = self.snapshot()
            if value['status'] == 'running': return value
            if value['status'] != 'starting': raise RuntimeError(value['message'])
            time.sleep(.05)
        self.stop()
        with self.lock: self.state.update(status='failed', message='巡迹节点启动超时，已结束本次进程')
        raise RuntimeError(self.state['message'])

    def stop(self):
        with self.lock:
            process = self.process
            self.state.update(status='stopping', message='正在停止视觉巡迹并关闭样本写入')
        try:
            if process and process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=10.)
                except ProcessLookupError: pass
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5.)
        except Exception:
            with self.lock:
                self.state.update(status='failed', message='巡迹进程停止未确认，请重试停止并检查日志')
            raise
        with self.lock:
            if self.log: self.log.close(); self.log = None
            self.process = None
            self.state.update(status='stopped', message='视觉巡迹已停止；连续采集状态不变')
            return self.snapshot()

    def save_sample(self, run_id=None):
        self.check_storage(); self.check_inputs()
        if self.snapshot()['status'] != 'running' or not self.snapshot()['diagnostics_fresh']:
            raise RuntimeError('巡迹节点未运行或诊断已过期')
        with self.lock: self.state['sample'] = {'status': 'saving', 'message': '等待下一组有效配对输入并写入原始样本'}
        try:
            value = self.service_call('save_snapshot')
            request_id = value['request_id']
            deadline = time.monotonic() + 12.
            while time.monotonic() < deadline:
                if self.snapshot()['status'] != 'running': raise RuntimeError('巡迹已停止，样本保存未确认')
                value = self.service_call('sample_status')
                if value.get('request_id') != request_id: raise RuntimeError('样本请求关联不一致')
                if value['status'] == 'failed': raise RuntimeError(value['message'])
                if value['status'] == 'completed':
                    relative = Path(value['path'])
                    directory = (self.session / 'samples' / relative).resolve()
                    if not directory.is_relative_to((self.session / 'samples').resolve()):
                        raise RuntimeError('非法样本路径')
                    meta = json.loads((directory / 'metadata.json').read_text())
                    if meta.get('complete') is not True: raise RuntimeError('样本尚未完整')
                    for f in meta['files']:
                        path = (directory / f['path']).resolve()
                        if not path.is_relative_to(directory) or hashlib.sha256(path.read_bytes()).hexdigest() != f['sha256']:
                            raise RuntimeError('样本文件校验失败')
                    # Link to a running MCAP without modifying the recorder or declaring labels.
                    write_json(directory / 'collection_reference.json', {'run_id': run_id, 'request_id': request_id,
                               'session_id': self.state['session_id'], 'annotation_status': 'unlabelled'})
                    index_path = self.session / 'index.json'
                    index = json.loads(index_path.read_text()) if index_path.exists() else {'schema': 'd405_dataset_v1', 'synthetic': False, 'samples': []}
                    record = dict(meta['sample'], sample=str((directory / meta['sample']['sample']).relative_to(self.session)))
                    index['samples'].append(record)
                    write_json(index_path, index)
                    with self.lock: self.state['saved_count'] = len(index['samples'])
                    value = dict(value, path=str(directory), message='实验样本已保存并通过文件校验；待人工标注', run_id=run_id)
                    with self.lock: self.state['sample'] = value
                    return value
                time.sleep(.1)
            raise TimeoutError('样本保存超时，结果未知；请检查本次实验目录')
        except Exception as error:
            with self.lock: self.state['sample'] = {'status': 'unknown' if isinstance(error, TimeoutError) or '超时' in str(error) else 'failed', 'message': str(error)}
            raise

    def close(self):
        self.stop()
