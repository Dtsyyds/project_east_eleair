"""Same decoded identity fixtures through validation, ROS payloads and RRD selection."""
from pathlib import Path
import importlib.util,sys,json
import pytest
ROOT=Path(__file__).resolve().parents[2]
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
    return module
sys.path.insert(0,str(ROOT/'physic_bev/scripts'))
validator=load('fixture_validator',ROOT/'physic_bev/scripts/validate_vehicle_collection.py')
replayer=load('fixture_replayer',ROOT/'physic_bev/scripts/replay_vehicle_collection.py')
video=load('fixture_video',ROOT/'physic_bev/src/inspection_tools/test/test_video_index.py')
# No temporary cross-repo implementation path: this import is from installed inspection_rerun.
from inspection_rerun.run_rrd_builder import discover_camera, select_camera_rows
from inspection_tools.video_index import create_frame_map
@pytest.mark.parametrize('indices',[[0,1,2,3,4],[1,2,3,4],[0,1,3,4]])
def test_consumer_mapping_is_identical(tmp_path,indices):
    v,s=video.fixture(tmp_path/'video/front',indices)
    create_frame_map(v,s,v.with_suffix('.frame_map.json'))
    validation=validator.validate_video(tmp_path)
    assert all(s['strict_alignment'] for s in validation['segments']),validation
    source=replayer.VideoSource(tmp_path);ros=[]
    while source.next_item is not None:
        stamp,topic,msg,type_name=source.next_item
        assert stamp==msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        assert type_name=='sensor_msgs/msg/Image' and msg.encoding=='bgr8' and msg.height==24
        ros.append(stamp);source.advance()
    segments=discover_camera(tmp_path)
    rows,_=select_camera_rows(segments[0],'full',10.)
    expected=[1000000000+i*100000000 for i in indices]
    assert ros==[r['ros_header_stamp_ns'] for r in rows]==expected

@pytest.mark.parametrize('case',['legacy','corrupt','cross_regression'])
def test_all_consumers_reject_unprovable_alignment(tmp_path,case):
    v,s=video.fixture(tmp_path/'video/front')
    if case!='legacy':create_frame_map(v,s,v.with_suffix('.frame_map.json'))
    if case=='corrupt':v.write_bytes(b'bad fixture video')
    if case=='cross_regression':
        import shutil
        for src in [v,s,v.with_suffix('.frame_map.json')]:
            dest=src.with_name(src.name.replace('000001','000002'));shutil.copy2(src,dest)
    report=validator.validate_video(tmp_path)
    assert not all(s['strict_alignment'] for s in report['segments'])
    with pytest.raises((ValueError,RuntimeError)):replayer.VideoSource(tmp_path)
    with pytest.raises((ValueError,RuntimeError)):discover_camera(tmp_path)


def test_replay_source_manifest_hashes_actual_selected_bytes(tmp_path):
    (tmp_path/'run_manifest.json').write_text('{"run_id":"fixture"}')
    v,s=video.fixture(tmp_path/'video/front')
    create_frame_map(v,s,v.with_suffix('.frame_map.json'))
    sources=[replayer.VideoSource(tmp_path)]
    hashes=replayer.source_hashes(tmp_path,sources)
    assert len(hashes)==4 and hashes['video/front/'+v.name]==replayer.sha256_file(v)
    assert replayer.source_hashes(tmp_path,[])=={'run_manifest.json':replayer.sha256_file(tmp_path/'run_manifest.json')}
    (tmp_path/'run_manifest.json').unlink()
    with pytest.raises(FileNotFoundError):replayer.source_hashes(tmp_path,sources)


def _write_mcap(path,topic,type_name,messages,stamps,additional=()):
    import gc,rosbag2_py
    from rclpy.serialization import serialize_message
    path.parent.mkdir(parents=True,exist_ok=True)
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    groups=[(topic,type_name,messages,stamps),*additional];records=[]
    for name,kind,values,times in groups:
        writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=kind,serialization_format='cdr'))
        records.extend((timestamp,name,serialize_message(message)) for message,timestamp in zip(values,times))
    for timestamp,name,payload in sorted(records,key=lambda r:(r[0],r[1])):writer.write(name,payload,timestamp)
    del writer;gc.collect()
    assert list(path.glob('*.mcap')) and (path/'metadata.yaml').is_file()

def test_actual_mcap_write_close_read_and_deserialize(tmp_path):
    from sensor_msgs.msg import Imu
    from rclpy.serialization import deserialize_message
    messages=[];stamps=[10**9+i*100000000 for i in range(5)]
    for index,timestamp in enumerate(stamps):
        m=Imu();m.header.stamp.sec,m.header.stamp.nanosec=divmod(timestamp,10**9);m.header.frame_id='fixture_imu'
        m.angular_velocity.x=index+.5;messages.append(m)
    bag=tmp_path/'mcap/core';_write_mcap(bag,'/livox/imu_raw','sensor_msgs/msg/Imu',messages,stamps)
    source=replayer.BagSource(bag);seen=[]
    while source.next_item:
        timestamp,topic,payload,type_name=source.next_item;m=deserialize_message(payload,Imu)
        seen.append((timestamp,topic,type_name,m.header.stamp.sec*10**9+m.header.stamp.nanosec,m.angular_velocity.x,m.header.frame_id));source.advance()
    assert seen==[(v,'/livox/imu_raw','sensor_msgs/msg/Imu',v,i+.5,'fixture_imu') for i,v in enumerate(stamps)]

@pytest.mark.parametrize('mode',['single','multi','multi_video','algorithm_video'])
def test_actual_mcap_original_rate_replay_to_independent_dds_subscriber(tmp_path,mode):
    import os,subprocess,time,hashlib,signal
    import rclpy
    from sensor_msgs.msg import CameraInfo,Imu,Image,PointCloud2
    from eddy_driver.msg import EddyChannelMean
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    run=tmp_path/'runs/FIXTURE-CLOSEOUT';stamps=[10**9+i*100000000 for i in range(5)]
    (run/'mcap').mkdir(parents=True);(run/'run_manifest.json').write_text(json.dumps({'run_id':run.name,'status':{'complete':True,'finalized':True}}))
    expected={};messages=[]
    for v in stamps:
        msg=CameraInfo(width=1,height=1);msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(v,10**9);msg.header.frame_id='fixture_camera';messages.append(msg)
    _write_mcap(run/'mcap/core','/camera/front/camera_info','sensor_msgs/msg/CameraInfo',messages,stamps)
    expected['/camera/front/camera_info']=(CameraInfo,stamps)
    if mode not in {'single','algorithm_video'}:
        for group,topic,cls,type_name in [('mid360','/livox/imu_raw',Imu,'sensor_msgs/msg/Imu'),('eddy','/eddy/channel_mean',EddyChannelMean,'eddy_driver/msg/EddyChannelMean'),('d405','/d405/camera/depth/image_rect_raw',Image,'sensor_msgs/msg/Image')]:
            values=[]
            for i,v in enumerate(stamps):
                msg=cls();msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(v,10**9);msg.header.frame_id='fixture_sensor'
                if group=='eddy':msg.channel_mean=[float(i)]*8
                elif group=='mid360':
                    msg.angular_velocity.x=i+.5;msg.linear_acceleration.x=i+.25;msg.linear_acceleration.y=-.5;msg.linear_acceleration.z=1.
                else:msg.height=1;msg.width=1;msg.encoding='16UC1';msg.step=2;msg.data=bytes([i,0])
                values.append(msg)
            additional=[]
            if group=='mid360':
                clouds=[]
                for i,v in enumerate(stamps):
                    cloud=PointCloud2(height=1,width=1,point_step=4,row_step=4,data=bytes([i,0,0,0]));cloud.header.stamp.sec,cloud.header.stamp.nanosec=divmod(v,10**9);cloud.header.frame_id='fixture_lidar';clouds.append(cloud)
                additional=[('/livox/lidar','sensor_msgs/msg/PointCloud2',clouds,stamps)]
                expected['/livox/lidar_points']=(PointCloud2,stamps)
            _write_mcap(run/'mcap'/group,topic,type_name,values,stamps,additional)
            expected[replayer.map_topic('sensor-only',topic)]=(cls,stamps)
    if mode in {'multi_video','algorithm_video'}:
        # Two normal segments; second run-local journal is shifted, not guessed.
        for segment in [1,2]:
            folder=run/'video/front' if segment==1 else tmp_path/'second_segment'
            v,s=video.fixture(folder)
            if segment==2:
                rows=[json.loads(x) for x in s.read_text().splitlines()]
                for row in rows:row['ros_header_stamp_ns']+=500000000
                s.write_text(''.join(json.dumps(r)+'\n' for r in rows))
                target=run/'video/front'/'front_000002.mkv';v.rename(target);s.rename(target.with_suffix('.frames.jsonl'));v=target;s=target.with_suffix('.frames.jsonl')
            create_frame_map(v,s,v.with_suffix('.frame_map.json'))
        expected['/camera/front/image_raw']=(Image,stamps+[v+500000000 for v in stamps])
    replay_mode='algorithm-replay' if mode=='algorithm_video' else 'full-sensor' if mode=='multi_video' else 'sensor-only'
    expected={replayer.map_topic(replay_mode,topic):value for topic,value in expected.items()}
    before={str(p.relative_to(run)):hashlib.sha256(p.read_bytes()).hexdigest() for p in run.rglob('*') if p.is_file()}
    os.environ['ROS_DOMAIN_ID']='201';os.environ['ROS_LOCALHOST_ONLY']='1'
    rclpy.init();node=rclpy.create_node('independent_replay_probe');received={name:[] for name in expected};matched={}
    qos=QoSProfile(depth=50,reliability=ReliabilityPolicy.RELIABLE)
    subs=[]
    for topic,(cls,_) in expected.items():
        subs.append(node.create_subscription(cls,topic,lambda msg,name=topic:received[name].append(msg),qos))
    report=tmp_path/'analysis/metrics';log=(tmp_path/'replay.log').open('w')
    command=[sys.executable,str(ROOT/'physic_bev/scripts/replay_vehicle_collection.py'),str(run),'--mode',replay_mode,'--rate','1.0','--report-dir',str(report)]
    child_env=os.environ.copy()
    if mode=='single':
        command=['bash',str(ROOT/'tank_inspection_system/scripts/replay.sh'),run.name,'sensor-only','1.0','0','0']
        child_env.update(FIELD_DATA_ROOT=str(tmp_path),FIELD_ANALYSIS_DIR=str(tmp_path/'analysis'))
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=child_env);deadline=time.monotonic()+15
    try:
        while time.monotonic()<deadline and process.poll() is None:
            rclpy.spin_once(node,timeout_sec=.02)
            for topic in expected:
                info=node.get_publishers_info_by_topic(topic)
                if info:matched[topic]={'types':[i.topic_type for i in info],'reliabilities':[int(i.qos_profile.reliability) for i in info]}
        assert process.poll()==0,(tmp_path/'replay.log').read_text()
        until=time.monotonic()+.3
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        counts={name:len(values) for name,values in received.items()}
        assert counts=={name:len(values) for name,(_,values) in expected.items()},counts
        for topic,(cls,stamps_expected) in expected.items():
            assert [m.header.stamp.sec*10**9+m.header.stamp.nanosec for m in received[topic]]==stamps_expected
            assert node.get_publishers_info_by_topic(topic)==[] or all(info.topic_type==cls.__module__.split('.')[0]+'/msg/'+cls.__name__ for info in node.get_publishers_info_by_topic(topic))
        assert set(matched)==set(expected),matched
        assert all(r['reliabilities']==[int(ReliabilityPolicy.RELIABLE)] for r in matched.values()),matched
        if '/d405/camera/depth/image_rect_raw' in received:assert [list(m.data) for m in received['/d405/camera/depth/image_rect_raw']]==[[i,0] for i in range(5)]
        if '/livox/imu' in received:
            assert [m.linear_acceleration.x for m in received['/livox/imu']]==pytest.approx([(i+.25)*9.80665 for i in range(5)])
            assert all(m.linear_acceleration.y==-.5*9.80665 and m.linear_acceleration.z==9.80665 and m.orientation_covariance[0]==-1. for m in received['/livox/imu'])
            assert [m.angular_velocity.x for m in received['/livox/imu']]==[i+.5 for i in range(5)]
        if '/eddy/channel_mean' in received:assert [m.channel_mean[0] for m in received['/eddy/channel_mean']]==list(map(float,range(5)))
        value=json.loads((report/'replay_validation.json').read_text());assert value['rate']==1.0 and value['topic_counts']==counts
        assert value['startup_discovery']['status']=='verified' and not value['startup_discovery']['data_receipt_verified']
        assert value['transport_acceptance']['status']=='not_executed'  # source report stays publisher-only
        after={str(p.relative_to(run)):hashlib.sha256(p.read_bytes()).hexdigest() for p in run.rglob('*') if p.is_file()};assert before==after
        print(json.dumps({'fixture':mode,'dds_received_counts':counts,'publisher_qos_types':matched,'rate':1.0,'source_sha_unchanged':True}))
    finally:
        node.destroy_node();rclpy.shutdown()
        if process.poll() is None:process.send_signal(signal.SIGINT);process.wait(timeout=3)
        log.close()


def test_actual_multi_bag_rrd_atomic_commit_and_role_handoff(tmp_path):
    import hashlib
    from eddy_driver.msg import EddyChannelMean
    from inspection_rerun.run_rrd_builder import RunBuilder,BuildOptions
    helper=load('closeout_rrd_helper',ROOT/'tank_vision_inspection/ros2_ws/src/inspection_rerun/test/test_run_rrd_builder.py')
    handoff=load('closeout_handoff',ROOT/'physic_bev/scripts/build_role_handoff.py')
    run,config=helper.create_fixture_run(tmp_path/'runs')
    stamps=[10**9+i*100000000 for i in range(5)];messages=[]
    for i,v in enumerate(stamps):
        m=EddyChannelMean(channel_mean=[float(i)]*8);m.header.stamp.sec,m.header.stamp.nanosec=divmod(v,10**9);messages.append(m)
    _write_mcap(run/'mcap/eddy','/eddy/channel_mean','eddy_driver/msg/EddyChannelMean',messages,stamps)
    manifest=json.loads((run/'run_manifest.json').read_text());manifest['enabled_writers']['eddy']=True
    for p in (run/'mcap/eddy').iterdir():
        if p.is_file():manifest['files'].append({'path':p.relative_to(run).as_posix(),'size':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
    (run/'run_manifest.json').write_text(json.dumps(manifest))
    validation=json.loads((run/'reports/mcap_validation.json').read_text())
    validation['bags']['eddy']=validator.validate_bag(run/'mcap/eddy',['/eddy/channel_mean'])
    assert validation['bags']['eddy']['valid']
    (run/'reports/mcap_validation.json').write_text(json.dumps(validation))
    before={p.relative_to(run).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in run.rglob('*') if p.is_file()}
    output=tmp_path/'analysis'/run.name/'fixture/outputs'/f'{run.name}.rrd'
    assert RunBuilder(BuildOptions(run,config,output,mode='strict',camera_mode='none')).run()==0
    report=json.loads((output.parent/'rrd_manifest.json').read_text())
    assert report['complete'] and report['hash_verified']
    assert report['topic_statistics']['/eddy/channel_mean']['source_count']==5
    assert report['topic_statistics']['/field/collection_status']['source_count']==3
    destination=tmp_path/'deliveries'/run.name/'rerun_viewer'
    args=['--run-dir',str(run),'--profile','rerun_viewer','--rrd-manifest',str(output.parent/'rrd_manifest.json'),'--output-root',str(destination.parent),'--destination',str(destination)]
    assert handoff.main(args+['--materialize','--copy-method','python'])==0
    assert handoff.main(args+['--verify-only'])==0
    assert list(destination.rglob('*.rrd'))
    after={p.relative_to(run).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in run.rglob('*') if p.is_file()};assert before==after
