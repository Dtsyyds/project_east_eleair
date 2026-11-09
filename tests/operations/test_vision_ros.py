"""Real ROS services/process/writer on domain 187 with synthetic input only."""
import importlib.util
import json
import os
from pathlib import Path
import threading
import time

import numpy as np
import pytest

from inspection_runtime.paths import project_root
from test_operations import request


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '187' or os.environ.get('ROS_LOCALHOST_ONLY') != '1', reason='explicit isolated ROS domain 187 required')
@pytest.mark.parametrize('capture_only', [False, True])
def test_isolated_ros_start_save_stop(tmp_path, capture_only):
    assert os.environ.get('ROS_DOMAIN_ID')=='187' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from realsense2_camera_msgs.msg import Extrinsics
    from std_msgs.msg import String
    import yaml
    from inspection_operations.operations_node import OperationsNode
    from d405_seam_tracking.cli import synthetic_frame
    from d405_seam_tracking.dataset import load_index, read_sample

    config=yaml.safe_load((project_root()/'config/communication/cameras.yaml').read_text())
    config['data_root']=str(tmp_path);config['minimum_free_bytes']=1
    config['required_topics']=['/d405/camera/color/image_raw','/d405/camera/depth/image_rect_raw']
    config['hardware']['start_mindvision']=False
    if capture_only:
        config['vision'] = {'python': '/missing/python', 'algorithm_config': '/missing/checkpoint.yaml'}
    rclpy.init()
    fixture=Node('vision_controls_isolated_fixture')
    events=[];ticks=[0]
    sub=fixture.create_subscription(String,'/inspection/operations/events',lambda m:events.append(json.loads(m.data)),64)
    pubs={}
    for key,topic,kind in [('image','color/image_raw',Image),('depth','depth/image_rect_raw',Image),
                           ('color_info','color/camera_info',CameraInfo),('depth_info','depth/camera_info',CameraInfo),
                           ('extrinsics','extrinsics/depth_to_color',Extrinsics)]:
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL) if key=='extrinsics' else qos_profile_sensor_data
        pubs[key]=fixture.create_publisher(kind,'/d405/camera/'+topic,qos)
    def publish():
        ticks[0]+=1
        frame=synthetic_frame(stamp=time.time_ns(),sequence=ticks[0]);h,w=frame.image.shape
        image=Image();image.header.frame_id='isolated_color_optical_frame'
        image.header.stamp.sec,image.header.stamp.nanosec=divmod(frame.image_stamp_ns,1_000_000_000)
        image.height=h;image.width=w;image.encoding='rgb8';image.step=w*3
        image.data=np.repeat(frame.image[:,:,None],3,axis=2).tobytes()
        depth=Image();depth.header=image.header;depth.height=h;depth.width=w;depth.encoding='16UC1';depth.step=w*2;depth.data=frame.depth.tobytes()
        info=CameraInfo();info.header=image.header;info.height=h;info.width=w;info.k=[float(v) for v in frame.image_calibration.k];info.distortion_model='none'
        ext=Extrinsics();ext.rotation=np.eye(3).ravel(order='F').tolist();ext.translation=[0.,0.,0.]
        for key,msg in [('color_info',info),('depth_info',info),('extrinsics',ext),('image',image),('depth',depth)]:pubs[key].publish(msg)
    timer=fixture.create_timer(.1,publish)
    node=OperationsNode(config)
    # Explicit test-only mount substitution; all writes stay in pytest's temporary directory.
    mounted=lambda:{'mounted':True,'free_bytes':10**10}
    node.operations.storage=mounted;node.operations.vision.storage=mounted
    executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node);executor.add_node(fixture)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    def wait(predicate,timeout=20):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if predicate():return
            time.sleep(.05)
        raise AssertionError('condition timed out; '+json.dumps(events[-3:],ensure_ascii=False))
    def result(ident):
        matches=[e['payload'] for e in events if e['payload'].get('request_id')==ident and e['payload'].get('status') in ('completed','failed')]
        return matches[-1] if matches else None
    try:
        wait(lambda:all(t in node.operations.live_topics() for t in config['required_topics']))
        node.operations.handle(request('vision.capture_start' if capture_only else 'vision.start','ros-start'))
        wait(lambda:result('ros-start'))
        assert result('ros-start')['status']=='completed',result('ros-start')
        tracking = None
        if not capture_only:
            wait(lambda:any('tracking_result' in e['payload'] for e in events))
            tracking=[e['payload']['tracking_result'] for e in events if 'tracking_result' in e['payload']][-1]
            assert tracking['frame_ref']['camera_id']=='depth_camera' and tracking['image_width']==160
        node.operations.handle(request('vision.save_sample','ros-save'))
        wait(lambda:result('ros-save'))
        assert result('ros-save')['status']=='completed',result('ros-save')
        saved=result('ros-save')['vision']['sample'];assert saved['status']=='completed'
        session=Path(result('ros-save')['vision']['directory'])
        root,index=load_index(session/'index.json');assert len(index['samples'])==1
        raw=read_sample(root,index['samples'][0]);assert raw.depth is not None and raw.source['stream']=='color'
        assert index['samples'][0]['annotation'] is None
        tick=ticks[0]
        node.operations.handle(request('vision.stop','ros-stop'));wait(lambda:result('ros-stop'))
        assert result('ros-stop')['status']=='completed'
        wait(lambda:ticks[0]>tick+2)  # Passive stop did not stop the camera fixture.
        assert json.loads((session/'samples/result_manifest.json').read_text())['complete'] is True
        if capture_only:
            from tools.vision.prepare_d405_review import prepare
            report = prepare(session/'index.json', tmp_path/'review')
            assert report['samples'] == 1
            review = json.loads((tmp_path/'review/review.json').read_text())
            assert review['samples'][0]['state'] == 'CAPTURE_ONLY'
            assert review['samples'][0]['points'] == []
            assert not any('tracking_result' in e['payload'] for e in events)
            assert node.operations.vision.snapshot()['capture_only'] is True
        evidence=project_root()/'.runtime/s100p/validation/vision_controls';evidence.mkdir(parents=True,exist_ok=True)
        (evidence/('capture-workflow.json' if capture_only else 'ros-workflow.json')).write_text(json.dumps({'isolated_ros_domain':187,'synthetic_input':True,
            'start':result('ros-start'),'save':result('ros-save'),'stop':result('ros-stop'),
            'tracking':tracking,'sample_index':index,'camera_continued':True},ensure_ascii=False,indent=2))
    finally:
        node.operations.close();executor.shutdown();thread.join(timeout=5)
        node.destroy_node();fixture.destroy_node();rclpy.shutdown()
