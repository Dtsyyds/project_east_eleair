from dataclasses import replace
import json
from pathlib import Path
import threading
import time
import numpy as np
import pytest
from d405_seam_tracking.cli import synthetic_frame, fixture
from d405_seam_tracking.contracts import PixelTransform, align_depth, rays, Calibration, load_config
from d405_seam_tracking.dataset import annotation_mask, save_sample, read_sample, split_check
from d405_seam_tracking.geometry import robust_lstsq, depth_evidence, roi_bounds
from d405_seam_tracking.pipeline import Pipeline
from d405_seam_tracking.writer import EventWriter

CONFIG = Path(__file__).resolve().parents[1] / 'configs/experimental.yaml'


@pytest.mark.parametrize('mode', ['vision_only', 'depth_only', 'fused'])
def test_branches(mode):
    p = Pipeline(dict(load_config(CONFIG), mode=mode))
    r = p.process(synthetic_frame())
    assert r['valid_2d'] and not r['predicted_only']
    assert r['metric_valid'] == (mode != 'vision_only')
    assert r['seam_identity_confirmed'] == (mode == 'fused')
    assert not r['body_coordinate_valid'] and r['unit_3d'] == 'metres'
    points = np.array(r['points_2d_y_left_right'])
    assert (np.diff(points[:, 0]) > 0).all() and (points[:, 1] < points[:, 2]).all()


@pytest.mark.parametrize('shape', [(120, 160), (480, 848)])
def test_visual_stripe_interior_remains_consistent_across_resolution(shape):
    result = Pipeline(load_config(CONFIG)).process(synthetic_frame(shape=shape))
    assert result['state'] == 'FUSED_CANDIDATE' and result['metric_valid']
    assert result['agreement_pixels'] < 2.


@pytest.mark.parametrize('case,state,metric', [('none', 'LOST', False), ('flat_visual', 'VISION_2D', False), ('invalid', 'VISION_2D', False), ('geometry_only', 'GEOMETRY_CANDIDATE', True), ('groove', 'FUSED_CANDIDATE', True), ('holes', 'VISION_2D', False), ('conflict', 'CONFLICT_2D', False)])
def test_evidence_degradation(case, state, metric):
    r = Pipeline(load_config(CONFIG)).process(synthetic_frame(case))
    assert r['state'] == state
    assert r['metric_valid'] == metric
    if case == 'geometry_only': assert not r['seam_identity_confirmed']


def test_source_time_prediction_reset_and_reacquisition():
    p = Pipeline(load_config(CONFIG)); f = synthetic_frame()
    assert p.process(f)['valid_2d']
    predicted = p.process(synthetic_frame('none', stamp=f.image_stamp_ns + 100_000_000))
    assert predicted['predicted_only'] and not predicted['metric_valid']
    assert p.process(synthetic_frame('none', stamp=f.image_stamp_ns + 300_000_000))['state'] == 'LOST'
    assert p.process(synthetic_frame(stamp=f.image_stamp_ns + 400_000_000))['state'] == 'FUSED_CANDIDATE'
    r = p.process(synthetic_frame('none', stamp=f.image_stamp_ns))
    assert 'source_time_duplicate_or_regression' in r['reasons'] and r['state'] == 'LOST'
    p.process(synthetic_frame(stamp=f.image_stamp_ns + 100_000_000))
    r = p.process(synthetic_frame('none', stamp=f.image_stamp_ns + 2_000_000_000))
    assert 'source_time_long_gap' in r['reasons'] and not r['predicted_only']


def test_missing_calibration_sync_and_resolution():
    p = Pipeline(load_config(CONFIG)); f = synthetic_frame()
    r = p.process(replace(f, depth_stamp_ns=f.image_stamp_ns + 30_000_000))
    assert r['valid_2d'] and not r['metric_valid'] and 'sync_tolerance_exceeded' in r['reasons']
    r = Pipeline(load_config(CONFIG)).process(replace(f, image_calibration=None))
    assert r['valid_2d'] and not r['metric_valid']
    with pytest.raises(ValueError, match='explicit_depth_scale'): replace(f, depth_scale_m=None).validate()
    with pytest.raises(ValueError, match='invalid_image_timestamp'): replace(f, image_stamp_ns=0).validate()
    p.process(f); r = p.process(synthetic_frame(shape=(144, 192), stamp=1_100_000_000))
    assert 'resolution_or_calibration_changed' in r['reasons']


def test_partial_metric_samples_do_not_claim_valid_3d_trajectory():
    f = synthetic_frame()
    measured = Pipeline(load_config(CONFIG)).process(f)
    pixels = np.asarray(measured['sampled_depth_pixels_xy'])
    # Remove actual sampled centres, rather than assuming a straight x=80 seam.
    missing = pixels[:int(len(pixels) * .8)]
    f.depth[missing[:, 1], missing[:, 0]] = 0; f.raw_valid = f.depth > 0
    r = Pipeline(load_config(CONFIG)).process(f)
    assert r['valid_2d'] and not r['metric_valid']
    assert 'trajectory_metric_coverage_low' in r['reasons']
    assert sum(r['points_3d_valid']) > 0 and any(p is None for p in r['points_3d_center'])


def test_units_projection_occlusion_and_invalid():
    f = synthetic_frame()
    a, m = align_depth(f, [.07, .6])
    floating = replace(f, depth=f.depth.astype(np.float32) * .001, depth_scale_m=1.)
    b, n = align_depth(floating, [.07, .6])
    np.testing.assert_allclose(a, b, atol=2e-8); np.testing.assert_array_equal(m, n)
    # Different intrinsic/size: both depth pixels project to one target pixel; nearer wins.
    src = Calibration(2, 1, [100, 0, .5, 0, 100, 0, 0, 0, 1], [], 'none', 'd', 'fixture')
    dst = Calibration(1, 1, [1, 0, 0, 0, 1, 0, 0, 0, 1], [], 'none', 'i', 'fixture')
    f = replace(f, image=np.zeros((1, 1), np.uint8), depth=np.array([[300, 200]], np.uint16), raw_valid=np.ones((1, 2), bool),
                image_calibration=dst, depth_calibration=src, depth_to_image={'rotation': np.eye(3).ravel().tolist(), 'translation_m': [0, 0, 0], 'from_frame': 'd', 'to_frame': 'i', 'source': 'fixture'})
    z, valid = align_depth(f, [.07, .6]); assert valid.item() and z.item() == pytest.approx(.2)
    f.depth = np.array([[np.nan, np.inf]], np.float32); f.depth_scale_m = 1.; z, valid = align_depth(f, [.07, .6])
    assert not valid.any() and np.isnan(z).all()
    f.depth_to_image['translation_m'] = [float('nan'), 0, 0]
    with pytest.raises(ValueError, match='invalid_rigid'): align_depth(f, [.07, .6])


def test_pixel_transform_and_distortion_roundtrip():
    t = PixelTransform(11, 17, .3, .7); p = np.array([[11, 17], [40.1, 68.2]])
    np.testing.assert_allclose(t.inverse(t.forward(p)), p, atol=1e-12)
    from d405_seam_tracking.contracts import project
    c = Calibration(640, 480, [440, 0, 320, 0, 440, 240, 0, 0, 1], [.1, -.03, .001, .002, 0], 'plumb_bob', 'c', 'fixture')
    p = np.array([[110., 220.], [340., 370.]])
    np.testing.assert_allclose(project(c, rays(c, p) * .3), p, atol=.001)


def test_robust_wall_and_degenerate_background():
    with pytest.raises(ValueError, match='fit_degenerate'): robust_lstsq(np.ones((100, 3)), np.zeros(100))
    f = synthetic_frame('none', shape=(240, 320)); y, x = np.mgrid[:240, :320]
    # Actual quadratic in camera X/Y with noise; no ridge -> no seam evidence.
    k = f.image_calibration; z = .3 + .003 * ((x - 160) / 320) ** 2
    f.depth = z.astype(np.float32); f.depth_scale_m = 1.
    r = Pipeline(load_config(CONFIG)).process(f)
    assert r['state'] == 'LOST'
    valid = np.zeros(f.depth.shape, bool); valid[130:132, 150:155] = True
    q, _ = depth_evidence(f.depth, valid, k, {}, load_config(CONFIG), roi_bounds(f.depth.shape, load_config(CONFIG)))
    assert not q['valid'] and q['reasons']


def test_dataset_hash_annotation_and_session_leak(tmp_path):
    f = synthetic_frame(); r = save_sample(tmp_path, f, 'sample', 's')
    np.testing.assert_array_equal(read_sample(tmp_path, r).depth, f.depth)
    with (tmp_path / r['sample']).open('ab') as stream: stream.write(b'tamper')
    with pytest.raises(ValueError, match='hash'): read_sample(tmp_path, r)
    annotation = {'status': 'approved', 'author': 'operator', 'origin': 'human', 'left_xy': [[20, 30], [25, 70]], 'right_xy': [[40, 30], [45, 70]]}
    m, rows = annotation_mask(annotation, f.image.shape); assert m.any() and rows[0][0] == 30
    annotation['right_xy'] = [[10, 30], [15, 70]]
    with pytest.raises(ValueError, match='crossed'): annotation_mask(annotation, f.image.shape)
    with pytest.raises(ValueError, match='session_leaks'): split_check([{'session': 's', 'split': 'train'}, {'session': 's', 'split': 'test'}])


def test_partial_annotations_do_not_label_unknown_rows_as_background():
    from d405_seam_tracking.dataset import label_valid_region
    from d405_seam_tracking.evaluation import Evaluator
    f = synthetic_frame(); r = Pipeline(load_config(CONFIG)).process(f)
    a = {'status': 'approved', 'author': 'operator', 'origin': 'human', 'left_xy': [[74, 45], [74, 90]], 'right_xy': [[85, 45], [85, 90]]}
    mask, rows = annotation_mask(a, f.image.shape)
    valid = label_valid_region(f.image.shape, rows)
    assert not valid[0].any() and valid[50].all() and not valid[100].any()
    probability = mask.astype(float); probability[~valid] = 1  # Unknown rows cannot become false positives.
    ev = Evaluator(); ev.add(f, r, a, probability)
    assert ev.report()['mask_iou']['p50'] == 1.


def test_requested_writer_complete_same_frame(tmp_path):
    f = synthetic_frame(); p = Pipeline(load_config(CONFIG)); r = p.process(f)
    w = EventWriter(tmp_path / 'events', maximum=2, interval_sec=0)
    assert w.request(f, r); w.close(.5); w.close(.5)
    manifest = json.loads((tmp_path / 'events/result_manifest.json').read_text())
    assert manifest['complete'] and not w.worker.thread.is_alive()
    directory = tmp_path / 'events' / manifest['events'][0]['path']
    saved = json.loads((directory / 'result.json').read_text()); assert saved['source']['image_stamp_ns'] == str(f.image_stamp_ns)
    from d405_seam_tracking.contracts import sha256
    for item in json.loads((directory / 'metadata.json').read_text())['files']:
        assert sha256(directory / item['path']) == item['sha256']


def test_three_mode_replay_includes_failure_frames(tmp_path):
    from d405_seam_tracking.cli import replay
    fixture(tmp_path / 'dataset')
    report = replay(tmp_path / 'dataset/index.json', CONFIG, tmp_path / 'analysis', ['vision_only', 'depth_only', 'fused'])
    assert set(report['modes']) == {'vision_only', 'depth_only', 'fused'}
    for mode, values in report['modes'].items():
        assert values['frames_total'] == values['labelled_frames'] == 24
        assert values['measured_frames'] + values['predicted_frames'] + values['lost_frames'] == 24
        assert values['millimetre_errors'].startswith('unavailable')
        assert len((tmp_path / 'analysis' / (mode + '.jsonl')).read_text().splitlines()) == 24


def test_writer_failure_full_queue_timeout(tmp_path):
    gate = threading.Event()
    def blocked(item): gate.wait(2.)
    f = synthetic_frame(); r = Pipeline(load_config(CONFIG)).process(f)
    w = EventWriter(tmp_path / 'timeout', capacity=1, interval_sec=0, process=blocked)
    w.request(f, r)
    deadline = time.monotonic() + .2
    while w.worker.queue.qsize() and time.monotonic() < deadline: time.sleep(.001)
    w.request(replace(f, sequence=1), r); assert not w.request(replace(f, sequence=2), r)
    start = time.monotonic()
    with pytest.raises(TimeoutError): w.close(.03)
    assert time.monotonic() - start < .15 and w.worker.status['worker_alive']
    assert not json.loads((tmp_path / 'timeout/result_manifest.json').read_text())['complete']
    gate.set(); w.worker.thread.join(.5)
    assert not w.worker.thread.is_alive() and w.worker.status['abandoned'] == 1
    def failing(item): raise OSError('injected')
    w = EventWriter(tmp_path / 'failure', process=failing); w.request(f, r)
    w.worker.thread.join(.5)
    with pytest.raises(RuntimeError, match='worker failed'): w.close(.1)
    assert not json.loads((tmp_path / 'failure/result_manifest.json').read_text())['complete']


def test_opencl_gpu_real_comparison():
    from d405_seam_tracking.acceleration import OpenCLGeometry
    try: gpu = OpenCLGeometry()
    except RuntimeError as exc: pytest.skip(str(exc))
    try:
        for shape in ((47, 83), (120, 160)):
            f = synthetic_frame(shape=shape); z = np.full((shape[0] * 2, shape[1] * 2), .3, np.float32)[::2, ::2]
            z[2, 3] = np.nan; mask = np.isfinite(z); mask[5, 7] = False
            c = f.image_calibration; coef = [.3, .001, .002, .0004, .0003, .0001]; origin = [.01, -.01]; scale = [.1, .08]
            bounds = (11, 17, 11 + shape[1], 17 + shape[0]); yy, xx = np.mgrid[:shape[0], :shape[1]]
            xyz = rays(c, np.column_stack(((xx + 11).ravel(), (yy + 17).ravel()))) * z.reshape(-1, 1)
            x, y = ((xyz[:, :2] - origin) / scale).T
            height = (coef[0] + coef[1] * x + coef[2] * y + coef[3] * x*x + coef[4] * x*y + coef[5]*y*y - xyz[:, 2]) / np.sqrt(1 + ((coef[1]+2*coef[3]*x+coef[4]*y)/scale[0])**2 + ((coef[2]+coef[4]*x+2*coef[5]*y)/scale[1])**2)
            expected = np.column_stack((xyz, height)).reshape(shape + (4,)); expected[~mask] = np.nan
            actual = gpu.compute(z, mask, c, bounds, coef, origin, scale)
            np.testing.assert_allclose(actual, expected, atol=2e-7, rtol=1e-5, equal_nan=True)
    finally: gpu.close()


def test_explicit_missing_models_fail_without_download():
    from d405_seam_tracking.learning import VisualModel
    from d405_seam_tracking.acceleration import BPUVisual
    with pytest.raises(RuntimeError, match='model_unavailable'): VisualModel({'visual_backend': 'smp', 'checkpoint': None})
    with pytest.raises(RuntimeError, match='seam HBM missing'): BPUVisual(None, None)
    m = VisualModel(dict(load_config(CONFIG), visual_backend='auto'))
    _, backend, reason = m.infer(synthetic_frame().image)
    assert backend == 'classical_cpu' and reason == 'model_unavailable_fallback_classical'


def test_projection_matches_installed_realsense_sdk_without_device():
    import ctypes as ct
    from d405_seam_tracking.contracts import project
    library = Path(__file__).resolve().parents[2] / 'third_party/librealsense/build/release/librealsense2.so.2.58.4'
    if not library.exists(): pytest.skip('matching RealSense SDK library unavailable')
    class Intrinsics(ct.Structure):
        _fields_ = [('width', ct.c_int), ('height', ct.c_int), ('ppx', ct.c_float), ('ppy', ct.c_float),
                    ('fx', ct.c_float), ('fy', ct.c_float), ('model', ct.c_int), ('coeffs', ct.c_float * 5)]
    sdk = ct.CDLL(str(library)); fp = ct.POINTER(ct.c_float)
    sdk.rs2_deproject_pixel_to_point.argtypes = [fp, ct.POINTER(Intrinsics), fp, ct.c_float]
    sdk.rs2_project_point_to_pixel.argtypes = [fp, ct.POINTER(Intrinsics), fp]
    for coefficients in ([0.] * 5, [.01, -.002, .0001, -.0002, .0003]):
        intr = Intrinsics(848, 480, 427.21936, 240.20366, 435.18689, 435.18689, 4, (ct.c_float * 5)(*coefficients))
        c = Calibration(848, 480, [intr.fx, 0, intr.ppx, 0, intr.fy, intr.ppy, 0, 0, 1], list(intr.coeffs), 'plumb_bob', 'fixture_optical', 'SDK_geometry_comparison')
        pixels = np.array([[210, 170], [427, 240], [600, 350]], float)
        expected, projected = [], []
        for pixel in pixels:
            point, output = (ct.c_float * 3)(), (ct.c_float * 2)()
            sdk.rs2_deproject_pixel_to_point(point, ct.byref(intr), (ct.c_float * 2)(*pixel), .3)
            sdk.rs2_project_point_to_pixel(output, ct.byref(intr), point)
            expected.append(list(point)); projected.append(list(output))
        np.testing.assert_allclose(rays(c, pixels) * .3, expected, atol=2e-7, rtol=1e-5)
        np.testing.assert_allclose(project(c, np.asarray(expected)), projected, atol=1e-4, rtol=1e-6)


def test_requested_overlay_panels_preserve_raw_input_and_result():
    from d405_seam_tracking.pipeline import overlay
    f = synthetic_frame('invalid', shape=(480, 848))
    original_image, original_depth = f.image.copy(), f.depth.copy()
    result, diagnostic = Pipeline(load_config(CONFIG)).process(f, diagnostic=True)
    frozen = json.dumps(result, sort_keys=True)
    view = overlay(f, result, diagnostic)
    assert view.shape == (480, 848, 3) and json.dumps(result, sort_keys=True) == frozen
    np.testing.assert_array_equal(f.image, original_image); np.testing.assert_array_equal(f.depth, original_depth)
    # The measured-mask panel stays black away from its label; display cannot fill holes.
    assert not view[230:300, 650:830].any()


@pytest.mark.parametrize('model,d', [
    ('none', []), ('plumb_bob', [0.] * 5),
    ('plumb_bob', [-.05372588336467743, .06000261381268501,
                   -.0006445523467846215, .00011522314889589325, -.020188620314002037]),
    ('plumb_bob', [.01, -.002, .0001, -.0002]),
    ('rational_polynomial', [.01, -.002, .0001, -.0002, .0003, .001, .002, .003]),
    ('equidistant', [0.] * 4),
])
def test_dense_projection_without_jacobian_matches_opencv(model, d):
    import cv2
    from d405_seam_tracking.contracts import project
    c = Calibration(848, 480, [434.66, 0, 421.9, 0, 433.54, 243.64, 0, 0, 1],
                    d, model, 'optical', 'projection_regression')
    yy, xx = np.mgrid[0:480:3, 0:848:3]
    pixels = np.column_stack((xx.ravel(), yy.ravel())).astype(float)
    k = np.array(c.k).reshape(3, 3); dist = np.array(d) if d else None
    if model == 'equidistant':
        reference_rays = cv2.fisheye.undistortPoints(pixels.reshape(-1, 1, 2), k, dist).reshape(-1, 2)
    else:
        reference_rays = cv2.undistortPoints(pixels.reshape(-1, 1, 2), k, dist).reshape(-1, 2)
    np.testing.assert_allclose(rays(c, pixels)[:, :2], reference_rays, atol=2e-15)
    points = np.column_stack((reference_rays, np.ones(len(pixels)))) * .3
    points[:, 0] += .001; points[:, 2] += .0005
    if model == 'equidistant':
        expected = cv2.fisheye.projectPoints(points.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), k, dist)[0].reshape(-1, 2)
    else:
        expected = cv2.projectPoints(points, np.zeros(3), np.zeros(3), k, dist)[0].reshape(-1, 2)
    actual = project(c, points[::2])  # Noncontiguous input is part of the contract.
    np.testing.assert_allclose(actual, expected[::2], atol=3e-12, rtol=1e-14)
    np.testing.assert_array_equal(np.rint(actual), np.rint(expected[::2]))
