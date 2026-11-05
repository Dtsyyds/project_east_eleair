#!/usr/bin/env python3
"""Isolated ROS candidate -> web operations -> selection ack -> tracking test.

Uses real ROS adapters and the actual recovery algorithm, with a synthetic
probability producer replacing only neural inference. No hardware or chassis.
Requires ROS_DOMAIN_ID=189 and ROS_LOCALHOST_ONLY=1.
"""
from array import array
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'packages/d405_seam_tracking/src'),
               str(ROOT/'ros2_ws/src/functions/perception/d405_seam_ros')]


def fixture_node():
    from d405_seam_tracking import round1_a, candidate_pipeline
    from d405_seam_ros.node import main
    class Visual:
        model_sha256 = 'synthetic-probability-fixture'
        last_transform = None
        def __init__(self, config): pass
        def infer(self, image): return image[:, :, 0].astype(np.float32)/255., 'fixture', None
        def close(self): pass
    round1_a.Round1AVisual = Visual
    candidate_pipeline.CandidateVisual = lambda original: original
    main(['recovery_fixture', '--config', str(ROOT/'config/d405/round1_a.yaml'),
          '--runtime-mode', 'candidate', '--stream', 'color', '--depth-scale-m', '.001',
          '--session-id', 'recovery-fixture'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture-node', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '189' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('requires isolated ROS_DOMAIN_ID=189 ROS_LOCALHOST_ONLY=1')
    if args.fixture_node: return fixture_node()
    if args.output is None: parser.error('--output required')
    args.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from duct_robot_msgs.msg import WeldTrackingError
    from inspection_operations.operations_node import OperationsNode
    import yaml

    rclpy.init()
    fixture = Node('isolated_recovery_test')
    config = yaml.safe_load((ROOT/'config/communication/d405_candidate.yaml').read_text())
    config.update(data_root=str(args.output), required_topics=['/d405/camera/color/image_raw'])
    operations = OperationsNode(config)
    operations.operations.vision.state.update(status='starting', session_id='recovery-fixture')
    executor = SingleThreadedExecutor()
    executor.add_node(fixture); executor.add_node(operations)
    candidates, errors, events, results = [], [], [], []
    fixture.create_subscription(String, '/weld_tracking/candidates', lambda m: candidates.append(json.loads(m.data)), 10)
    fixture.create_subscription(WeldTrackingError, '/weld_tracking/error', errors.append, qos_profile_sensor_data)
    fixture.create_subscription(String, '/inspection/operations/events', lambda m: events.append(json.loads(m.data)['payload']), 64)
    fixture.create_subscription(String, '/vision/d405_seam/result', lambda m: results.append(json.loads(m.data)), 10)
    images = fixture.create_publisher(Image, '/d405/camera/color/image_raw', qos_profile_sensor_data)
    commands = fixture.create_publisher(String, '/inspection/operations/command', 10)
    stripes = [(220, .45), (320, .40), (420, .35)]
    publishing = [True]
    def publish():
        if not publishing[0]: return
        pixels = np.zeros((480,640,3), np.uint8)
        for x, value in stripes: pixels[168:456,x-5:x+6] = round(value*255)
        m = Image(); m.header.frame_id = 'synthetic_optical_frame'
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(time.time_ns(), 1_000_000_000)
        m.width=640; m.height=480; m.encoding='rgb8'; m.step=640*3
        m.data=array('B',pixels.tobytes()); images.publish(m)
    fixture.create_timer(.1, publish)
    child = None
    def wait(predicate, timeout=12):
        deadline=time.monotonic()+timeout
        while not predicate():
            if child and child.poll() is not None:
                raise RuntimeError('fixture exited: '+(args.output/'node.log').read_text()[-3000:])
            if time.monotonic()>deadline: raise TimeoutError('ROS recovery fixture timed out')
            executor.spin_once(timeout_sec=.03)
    def response(ident):
        return next((e for e in reversed(events) if e.get('request_id')==ident and e.get('status') in ('completed','failed')), None)
    def command(action, ident, parameters):
        m=String();m.data=json.dumps(dict(action=action,request_id=ident,client_id='isolated-test',parameters=parameters))
        commands.publish(m);wait(lambda:response(ident) is not None)
        return response(ident)
    report = {'hardware_started':False,'synthetic_inference':True,'ros_domain_id':189}
    try:
        with (args.output/'node.log').open('w') as log:
            child=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--fixture-node'],stdout=log,stderr=subprocess.STDOUT)
            wait(lambda:candidates and len(candidates[-1]['candidates'])==3 and operations.operations.vision.snapshot()['diagnostics_fresh'])
            assert errors and not errors[-1].valid
            offer=candidates[-1]
            parameters=dict(session_id='recovery-fixture',candidate_set_id=offer['candidate_set_id'],candidate_id='1')
            assert command('vision.select_candidate','choose',parameters)['status']=='completed'
            stripes[:]=[(228,.4),(420,.95)]
            wait(lambda:errors and errors[-1].valid and abs(errors[-1].lateral_error_norm-(228-320)/320)<1e-5)
            assert results[-1]['selection']['state']=='TRACKING'
            report['selected_target_follows_new_image']=True
            report['stronger_other_line_does_not_steal_selection']=True
            # Old batch ids cannot be reused as a new choice after reacquisition.
            assert command('vision.select_candidate','expired',parameters)['status']=='failed'
            stripes[:]=[(420,.95)]
            wait(lambda:results[-1]['selection']['state']=='RESELECT_REQUIRED' and not errors[-1].valid)
            assert len(candidates[-1]['candidates'])==1
            report['selected_loss_requires_reselection']=True
            assert command('vision.clear_selection','clear',{'session_id':'recovery-fixture'})['status']=='completed'
            wait(lambda:errors[-1].valid and errors[-1].lateral_error_norm>0 and results[-1]['selection']['state']=='AUTO')
            report['clear_restores_automatic_A']=True
            stripes[:]=[]
            wait(lambda:not candidates[-1]['candidates'] and not errors[-1].valid)
            report['blank_image_does_not_fabricate_candidates']=True
            report['frames_processed']=len(results)
            report['operations_adapter_source']=sys.modules['inspection_operations.operations_node'].__file__
    finally:
        operations.operations.close()
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT);child.wait(timeout=10)
        executor.shutdown();operations.destroy_node();fixture.destroy_node();rclpy.shutdown()
    assert child.returncode==0
    report['passed']=True
    (args.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__': main()
