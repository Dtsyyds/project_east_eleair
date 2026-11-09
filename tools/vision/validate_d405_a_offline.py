#!/usr/bin/env python3
"""Existing samples + complete existing MCAP through the passive ROS A entry.

Requires isolated localhost domain 187. Never launches camera/hardware/control.
The replay recording uses the real Operations.start/stop MCAP writer, labelled
回放测试. Original header stamps and RGB bytes are retained; pacing is deliberately
slower than capture to await each CPU result, so this is not a live-rate test.
"""
import argparse
from array import array
import csv
from dataclasses import replace
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'packages/d405_seam_tracking/src'),
             str(ROOT/'packages/inspection_runtime/src'),str(ROOT/'packages/emat_core/src'),
             str(ROOT/'experiments/vision/scripts')]
import cv2
import numpy as np
import yaml
from d405_seam_tracking.contracts import Frame, load_config, sha256, dump
from d405_seam_tracking.dataset import load_index, read_sample, image_array, stamp_ns
from d405_seam_tracking.round1_a import Round1AVisual
from d405_unet_round1 import result_from_probability
from inspection_runtime.vision import tracking_payload
from inspection_runtime.operations import Operations


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--bag',type=Path,default=Path('/media/mcap_data/runs/WS-20261028_170715-71283257/bag'))
    args=p.parse_args();out=args.output.resolve()
    if os.environ.get('ROS_DOMAIN_ID')!='187' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('requires ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1')
    out.mkdir(parents=True,exist_ok=False)
    old=ROOT/'.runtime/s100p/analysis/d405_unet_round1_20261102'
    preserved={str(f.relative_to(old)):sha256(f) for f in old.rglob('*') if f.is_file()}
    for e in json.loads((old/'delivery_manifest.json').read_text())['files']:
        assert preserved[e['path']]==e['sha256']
    cfg=load_config(ROOT/'config/d405/round1_a.yaml')
    original_cfg=json.loads((old/'visual_config.json').read_text())
    root,index=load_index(old/'dataset/index.json')
    manifest=json.loads((old/'model_manifest.json').read_text())
    archived=[json.loads(s) for s in (old/'model.jsonl').read_text().splitlines()]
    import rclpy
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.duration import Duration
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from nav_msgs.msg import Path as RosPath
    rclpy.init();fixture=Node('d405_a_offline_input_only')
    color_topic='/d405/camera/color/image_raw'
    result_topic='/vision/d405_seam/result'
    # Reliable offline publisher lets rosbag's adaptive subscription acknowledge
    # every source frame. It remains compatible with the node's BEST_EFFORT input.
    # This does not alter camera-driver QoS or claim live transport losslessness.
    publisher=fixture.create_publisher(Image,color_topic,QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE))
    received={};paths={};diagnostics=[]
    fixture.create_subscription(String,result_topic,lambda m:received.update({json.loads(m.data)['source']['image_stamp_ns']:json.loads(m.data)}),10)
    fixture.create_subscription(RosPath,'/vision/d405_seam/path_3d',lambda m:paths.update({str(stamp_ns(m)):len(m.poses)}),10)
    fixture.create_subscription(String,'/vision/d405_seam/diagnostics',lambda m:diagnostics.append(json.loads(m.data)),10)
    process=None;log=None;rows=[];clear_transitions=0;last_valid=False
    def wait(predicate,timeout=45):
        deadline=time.monotonic()+timeout
        while not predicate():
            if process is not None and process.poll() is not None:
                raise RuntimeError('passive node exited: '+log.name)
            if time.monotonic()>deadline:raise TimeoutError('offline ROS wait')
            rclpy.spin_once(fixture,timeout_sec=.05)
    def stop_node():
        nonlocal process,log
        if process:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=20)
            assert process.returncode==0
            process=None
        if log:log.close();log=None
    def start_node(config,name):
        nonlocal process,log
        diagnostics.clear();received.clear();paths.clear()
        config_path=out/(name+'.yaml');config_path.write_text(yaml.safe_dump(config))
        log=(out/(name+'.log')).open('w')
        process=subprocess.Popen([sys.executable,str(ROOT/'tools/vision/run_d405_node.py'),
             '--config',str(config_path),'--stream','color','--depth-scale-m','.001',
             '--session-id','OFFLINE-'+name,'--output',str(out/name)],stdout=log,stderr=subprocess.STDOUT)
        wait(lambda:publisher.get_subscription_count()>0 and any(d['session_id']=='OFFLINE-'+name for d in diagnostics))
    def message(frame):
        m=Image();m.header.frame_id=frame.source.get('image_frame_id','offline_color_optical_frame')
        m.header.stamp.sec,m.header.stamp.nanosec=divmod(frame.image_stamp_ns,1_000_000_000)
        m.height,m.width=frame.image.shape[:2];m.encoding='rgb8';m.step=m.width*3
        m.data=array('B',frame.image.tobytes());return m
    def check(m,expected,identifier,fold):
        nonlocal last_valid,clear_transitions
        stamp=str(stamp_ns(m));received.pop(stamp,None);paths.pop(stamp,None)
        publisher.publish(m)
        assert publisher.wait_for_all_acked(Duration(seconds=10))
        wait(lambda:stamp in received and stamp in paths)
        actual=received[stamp]
        assert actual['valid_2d']==expected['valid_2d']
        assert actual['state']==('VISION_2D' if expected['valid_2d'] else 'LOST')
        assert actual['predicted_only'] is False
        assert actual['points_2d_y_left_right']==expected['points_2d_y_left_right']
        assert actual['visual']['coverage']==expected['coverage']
        assert paths[stamp]==0  # A is pixel geometry, never metres.
        payload=tracking_payload(actual,m.width)
        pts=np.asarray(expected['points_2d_y_left_right'])
        if len(pts):
            np.testing.assert_allclose(payload['centerline'],np.column_stack((pts[:,1:].mean(1),pts[:,0])),rtol=0,atol=1e-12)
            np.testing.assert_allclose(payload['left_boundary'],pts[:,[1,0]],rtol=0,atol=1e-12)
            np.testing.assert_allclose(payload['right_boundary'],pts[:,[2,0]],rtol=0,atol=1e-12)
        else:
            assert payload['centerline']==payload['left_boundary']==payload['right_boundary']==[]
            if last_valid:clear_transitions+=1
        last_valid=actual['valid_2d']
        rows.append(dict(id=identifier,fold=fold,stamp_ns=stamp,state=actual['state'],
                         boundaries_exact=True,centerline_exact=True,stale_points=False,
                         model_sha256=actual['model_sha256'],processing_ms=actual['timing_ms']['total']))
        return actual
    ops=None
    try:
        for fold in manifest['models']:
            assert not set(fold['held_ids']) & set(fold['train_ids'])
            assert sha256(old/fold['weights'])==fold['sha256']
            selected=dict(cfg,checkpoint=str(old/fold['weights']),checkpoint_sha256=fold['sha256'])
            offline=Round1AVisual(selected)
            start_node(selected,'samples_fold_'+str(fold['fold']))
            for i in fold['held_ids']:
                frame=read_sample(root,index['samples'][i-1])
                probability,_,_=offline.infer(frame.image)
                np.testing.assert_array_equal(probability,np.load(old/'probabilities'/f'{i:02d}.npy',allow_pickle=False))
                expected=result_from_probability(frame.image,frame.image_stamp_ns,probability,original_cfg)
                assert all(expected[k]==archived[i-1][k] for k in expected)
                actual=check(message(frame),expected,i,fold['fold'])
                assert actual['model_sha256']==fold['sha256']
            stop_node();offline.close()
            print('Discrete held-out fold',fold['fold'],'exact; no video synthesized.',flush=True)
        dump(out/'discrete_30_results.json',rows)
        # Replay the ENTIRE existing color sequence, never stitch discrete samples.
        bag=args.bag.resolve();metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        registration=json.loads((bag.parent/'run.json').read_text())
        assert registration['complete'] is True
        source_counts={r['topic_metadata']['name']:r['message_count'] for r in metadata['topics_with_message_count']}
        hashes={str(f):sha256(f) for f in [bag/'metadata.yaml',bag.parent/'run.json']+[bag/f for f in metadata['relative_file_paths']]}
        reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
        reader.set_filter(rosbag2_py.StorageFilter(topics=[color_topic]))
        selected=dict(cfg);offline=Round1AVisual(selected)
        start_node(selected,'replay_a')
        live={};replay_config=yaml.safe_load((ROOT/'config/communication/d405_a.yaml').read_text())
        replay_config.update(data_root=str(out/'replay_test'),minimum_free_bytes=1,
             required_topics=[color_topic],record_topics=[color_topic,result_topic,'/vision/d405_seam/path_3d','/vision/d405_seam/diagnostics'])
        ops=Operations(replay_config,lambda _:None,lambda:dict(live))
        # This invokes the actual recorder implementation. Storage/mount preflight
        # is outside this offline lifecycle test; all writes stay in the output.
        assert ops.recorder is None and ops.task['status']=='idle'
        ops.start({'profile':'d405_experiment','task_name':'回放测试','operator_note':'已有完整MCAP逐帧回放；非实拍；源时间戳不变，CPU等待结果后发送下一帧'})
        wait(lambda:publisher.get_subscription_count()>=2)
        wait(lambda:any(info.node_name=='rosbag2_recorder' and info.qos_profile.reliability==ReliabilityPolicy.RELIABLE
                        for info in fixture.get_subscriptions_info_by_topic(color_topic)))
        replay=[];last=None
        while reader.has_next():
            topic,data,storage_time=reader.read_next();m=deserialize_message(data,Image)
            rgb=image_array(m);stamp=stamp_ns(m)
            assert m.encoding.lower()=='rgb8' and (last is None or stamp>last)
            last=stamp
            probability,_,_=offline.infer(rgb)
            expected=result_from_probability(rgb,stamp,probability,original_cfg)
            live[color_topic]=time.monotonic()
            actual=check(m,expected,'replay_'+str(len(replay)),1)
            import hashlib
            replay.append(dict(frame_index=len(replay),source_stamp_ns=str(stamp),source_bag_time_ns=str(storage_time),
                               rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),shape=list(rgb.shape),
                               state=actual['state'],model_sha256=actual['model_sha256']))
            if len(replay)%25==0:print('回放测试:',len(replay),'/',source_counts[color_topic],flush=True)
        assert len(replay)==source_counts[color_topic]
        # Let DDS deliver the last frame/result before the normal writer close.
        end=time.monotonic()+.5
        while time.monotonic()<end:rclpy.spin_once(fixture,timeout_sec=.05)
        live[color_topic]=time.monotonic()
        ops.stop();assert ops.task['complete'] is True
        run=ops.run_dir
        # Decode every recorded color and result; match actual bytes and source stamps.
        saved=rosbag2_py.SequentialReader();saved.open(rosbag2_py.StorageOptions(uri=str(run/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
        images=[];result_stamps=[];import hashlib
        while saved.has_next():
            topic,data,_=saved.read_next()
            if topic==color_topic:
                m=deserialize_message(data,Image);rgb=image_array(m)
                images.append((str(stamp_ns(m)),hashlib.sha256(rgb.tobytes()).hexdigest()))
            elif topic==result_topic:
                value=json.loads(deserialize_message(data,String).data)
                result_stamps.append(value['source']['image_stamp_ns'])
        dump(out/'recording_decode_audit.json',dict(expected=replay,decoded_images=images,decoded_result_stamps=result_stamps))
        assert images==[(r['source_stamp_ns'],r['rgb_sha256']) for r in replay]
        assert result_stamps==[r['source_stamp_ns'] for r in replay]
        assert process.poll() is None  # Stopping recording leaves preview/algorithm running.
        for path,digest in hashes.items():assert sha256(path)==digest
        dump(out/'replay_test_manifest.json',dict(label='回放测试',hardware_test=False,source_bag=str(bag),
             source_complete=True,source_hashes=hashes,full_color_sequence=True,frames=len(replay),
             capture_duration_seconds=metadata['duration']['nanoseconds']/1e9,recorded_run=str(run),
             recording_start_stop_complete=True,decoded_all_frames=True,source_rgb_bytes_exact=True,
             source_header_stamps_exact=True,result_header_stamps_exact=True,
             original_rate_tested=False,pace='one original frame, await CPU result, then next; no source restamping',
             replay_transport='RELIABLE publisher and adaptive recorder; production camera QoS unchanged',
             preview_started_without_recorder=True,preview_alive_after_recording_stop=True,
             run=json.loads((run/'run.json').read_text()),frames_mapping=replay))
        stop_node();offline.close();ops.close();ops=None
        assert {str(f.relative_to(old)):sha256(f) for f in old.rglob('*') if f.is_file()}==preserved
        dump(out/'summary.json',dict(discrete_held_out_frames_exact=30,replay_frames_exact=len(replay),
             no_measurement_cleared_transitions=clear_transitions,original_artifacts_unchanged=True,
             no_camera_or_chassis_started=True,hardware_tests='待验证',
             timing_note='CPU paced offline replay; not hardware real-time acceptance',
             python=sys.executable,ros_domain_id=187,localhost_only=True))
        with (out/'parity.csv').open('x') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    finally:
        if ops:ops.close()
        stop_node();fixture.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
