"""Real loopback WebSocket + installed C++ bridge + ROS DDS, no device drivers."""
import base64,json,math,os,signal,socket,subprocess,time,yaml
from pathlib import Path
import numpy as np
import cv2
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Pose2D
from std_msgs.msg import String,Float64MultiArray
from eddy_driver.msg import EddyChannelMean,EddyFrame
from inspection_interfaces.msg import UltrasoundFrame
from websockets.sync.client import connect

STAMP_BASE=1791029236000000000

def stamp(msg,value):
    msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(value,10**9);msg.header.frame_id='fixture_sensor'
    return msg

def recv_until(ws,predicate,timeout=4):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        try:msg=json.loads(ws.recv(timeout=.2))
        except TimeoutError:continue
        if predicate(msg):return msg
    raise AssertionError('missing expected WebSocket response/frame')

def command(ws,payload,kind='command',robot='tank_inspection_robot'):
    ws.send(json.dumps({'header':{'robot_id':robot,'msg_type':kind},'payload':payload}))
    return recv_until(ws,lambda m:m['header']['msg_type']=='response')['payload']

@pytest.mark.parametrize('control',[False,'geometry_msgs/msg/Twist','std_msgs/msg/Float64MultiArray'])
def test_real_wireless_bridge_devices_streams_pose_controls_and_exit(tmp_path,control):
    with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    binary=Path(get_package_prefix('agv_bridge'))/'lib/agv_bridge/websocket_bridge_node'
    params={'robot_id':'tank_inspection_robot','server_uri':'','local_server_port':port,
            'tank_inspection.fisheye_topic':'/fixture/fisheye','tank_inspection.mindvision_topic':'/fixture/mindvision',
            'tank_inspection.depth_camera_topic':'/fixture/depth','tank_inspection.eddy_raw_topic':'/fixture/eddy',
            'tank_inspection.eddy_mean_topic':'/fixture/eddy_mean','tank_inspection.emat_topic':'/fixture/emat',
            'tank_inspection.chassis_status_topic':'/fixture/chassis','tank_inspection.chassis_status_type':'geometry_msgs/msg/Pose2D' if control=='std_msgs/msg/Float64MultiArray' else 'std_msgs/msg/String','tank_inspection.lidar_topic':'/fixture/lidar',
            'tank_inspection.lidar_pose_topic':'/fixture/pose','tank_inspection.control_enabled':bool(control),
            'tank_inspection.chassis_command_topic':'/fixture/cmd','tank_inspection.device_timeout_sec':1.5,
            'tank_inspection.control_token':'fixture-token' if control=='geometry_msgs/msg/Twist' else '',
            'tank_inspection.command_timeout_sec':.2,'tank_inspection.chassis_command_type':control or 'geometry_msgs/msg/Twist'}
    config=tmp_path/'params.yaml';config.write_text(yaml.safe_dump({'websocket_bridge_node':{'ros__parameters':params}}))
    argv=([str(binary),'--ros-args','--params-file',str(config)] if control else
          ['ros2','launch','agv_bridge','tank_inspection.launch.py','config:='+str(config)])
    log=(tmp_path/'bridge.log').open('w');proc=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,env=dict(os.environ,ROS_LOG_DIR=str(tmp_path/'ros_logs')))
    rclpy.init();node=rclpy.create_node('wireless_fixture');received=[]
    status_class=Pose2D if control=='std_msgs/msg/Float64MultiArray' else String
    def chassis_msg():return Pose2D(x=1.,y=2.,theta=.3) if status_class==Pose2D else String(data='online')
    topics={'fisheye':Image,'mindvision':Image,'depth':Image,'eddy':EddyFrame,'eddy_mean':EddyChannelMean,'emat':UltrasoundFrame,'chassis':status_class,'lidar':PointCloud2,'pose':Odometry}
    pubs={name:node.create_publisher(cls,'/fixture/'+name,qos_profile_sensor_data) for name,cls in topics.items()}
    cls=Float64MultiArray if control=='std_msgs/msg/Float64MultiArray' else Twist
    node.create_subscription(cls,'/fixture/cmd',received.append,10)
    ws=None
    try:
        until=time.monotonic()+6
        while time.monotonic()<until:
            try:ws=connect(f'ws://127.0.0.1:{port}',proxy=None,open_timeout=.3);break
            except (OSError,TimeoutError):time.sleep(.05)
        assert ws is not None and proc.poll() is None
        required=['fisheye','mindvision','depth_camera','eddy_current','emat','chassis']
        initial=command(ws,{},'check');assert all(initial[k]['status']=='unverified' for k in required)
        mismatch=command(ws,{},'check','unregistered_robot');assert mismatch['error']['reason']=='robot_id_mismatch'
        until=time.monotonic()+4
        while time.monotonic()<until and not all(p.get_subscription_count() for p in pubs.values()):rclpy.spin_once(node,timeout_sec=.02)
        assert all(p.get_subscription_count() for p in pubs.values())
        # Discovery only; playback/publish pacing below is fixed, never slowed to pass.
        time.sleep(.3)
        for i in range(5):
            value=STAMP_BASE+i*50_000_000
            for key in ['fisheye','mindvision']:
                pubs[key].publish(stamp(Image(height=8,width=8,encoding='rgb8',step=24,data=[90,20,10]*64),value))
            pubs['depth'].publish(stamp(Image(height=8,width=8,encoding='16UC1',step=16,data=np.full((8,8),1000,dtype=np.uint16).tobytes()),value))
            pubs['eddy'].publish(stamp(EddyFrame(sequence=i,channel_count=8,samples_per_channel=20,data=list(range(160))),value))
            pubs['eddy_mean'].publish(stamp(EddyChannelMean(channel_mean=[float(i)]*8),value))
            pubs['emat'].publish(stamp(UltrasoundFrame(channel_count=1,sample_count=2,sampling_rate_hz=1000.,samples=[3,4]),value))
            pubs['chassis'].publish(chassis_msg())
            pubs['lidar'].publish(stamp(PointCloud2(height=1,width=1,point_step=4,row_step=4,data=b'\0'*4),value))
            pose=stamp(Odometry(),value);pose.child_frame_id='body';pose.pose.pose.position.x=1.25
            pose.pose.pose.orientation.z=math.sqrt(.5);pose.pose.pose.orientation.w=math.sqrt(.5);pubs['pose'].publish(pose)
            until=time.monotonic()+.05
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        checked=command(ws,{},'check');assert all(checked[k]['status']=='verified' for k in required),checked
        for key in required:
            assert checked[key]['device_id']==key and checked[key]['connection_state']=='connected'
            assert checked[key]['running_state']=='running' and checked[key]['data_state']=='streaming'
            assert checked[key]['publish_rate_hz']>0 and not checked[key]['error_code']
        ws.send(json.dumps({'header':{'robot_id':'tank_inspection_robot','msg_type':'command',
                    'schema_version':'tank_inspection_v2','request_id':'fixture-request-1'},
                    'payload':{'lidar':{'command':'GetStatus'}}}))
        correlated=recv_until(ws,lambda m:m['header'].get('request_id')=='fixture-request-1')
        assert correlated['header']['schema_version']=='tank_inspection_v2'
        assert correlated['payload']['lidar']['accepted']
        blocked=command(ws,{'lidar':{'command':'Restart'}})['lidar']
        assert not blocked['accepted'] and blocked['reason']=='passive_bridge_does_not_own_device'
        # Explicit new input after check response: check filtering may consume an
        # earlier preview, which must not be mistaken for transport loss.
        pubs['fisheye'].publish(stamp(Image(height=8,width=8,encoding='rgb8',step=24,data=[90,20,10]*64),STAMP_BASE+300_000_000))
        pubs['mindvision'].publish(stamp(Image(height=8,width=8,encoding='rgb8',step=24,data=[10,20,90]*64),STAMP_BASE+300_000_000))
        pubs['depth'].publish(stamp(Image(height=8,width=8,encoding='16UC1',step=16,data=np.full((8,8),1000,dtype=np.uint16).tobytes()),STAMP_BASE+300_000_000))
        pubs['eddy'].publish(stamp(EddyFrame(sequence=99,channel_count=8,samples_per_channel=20,data=list(range(160))),STAMP_BASE+300_000_000))
        pubs['eddy_mean'].publish(stamp(EddyChannelMean(channel_mean=[9.0]*8),STAMP_BASE+300_000_000))
        frames={};eca=None
        until=time.monotonic()+2
        while (len(frames)<3 or eca is None) and time.monotonic()<until:
            try:packet=json.loads(ws.recv(timeout=.3))
            except TimeoutError:continue
            if 'video' in packet['payload']:
                v=packet['payload']['video'];frames[v['camera_id']]=v
            if 'eca' in packet['payload']:eca=packet['payload']['eca']
        assert set(frames)=={'fisheye','mindvision','depth_camera'}
        for v in frames.values():
            decoded=cv2.imdecode(np.frombuffer(base64.b64decode(v['data']),np.uint8),cv2.IMREAD_UNCHANGED)
            assert decoded is not None and decoded.shape[:2]==(v['height'],v['width'])
            assert isinstance(v['stamp_ns'],str) and int(v['stamp_ns'])>=STAMP_BASE and int(v['stamp_ns'])==v['stamp_sec']*10**9+v['stamp_nanosec'] and v['frame_id']=='fixture_sensor' and v['visualization_only']
        assert eca is not None
        assert eca['raw']['channel_count']==8 and eca['raw']['samples_per_channel']==20
        assert len(eca['raw']['data'])==160 and len(eca['mean']['channel_mean'])==8
        report=recv_until(ws,lambda m:'lidar' in m['payload'])['payload']['lidar']
        assert report['x']==1.25 and math.isclose(report['angle'],math.pi/2) and report['angle_unit']=='rad'
        assert report['frame_id']=='fixture_sensor' and not report['asset_coordinate_valid']
        pubs['chassis'].publish(chassis_msg());time.sleep(.05)
        move={'agv':{'command':'move','parameters':{'vx':.1,'vy':0.,'wz':.1}}}
        if control=='geometry_msgs/msg/Twist':
            assert command(ws,move)['agv']['reason']=='invalid_control_token'
            move['token']='fixture-token'
        result=command(ws,move)
        if not control:assert result['agv']['reason']=='control_disabled' and not received
        else:
            assert result['agv']['execution']=='published_not_hardware_acknowledged'
            until=time.monotonic()+.8
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
            velocities=[tuple(m.data) if isinstance(m,Float64MultiArray) else (m.linear.x,m.linear.y,m.angular.z) for m in received]
            assert (.1,0.,.1) in velocities and velocities[-1]==(0.,0.,0.),velocities
            move['agv']['parameters']['vx']=9.
            out=command(ws,move)
            assert out['agv']['reason']=='velocity_out_of_bounds'
            stop={'agv':{'command':'stop'}}
            if control=='geometry_msgs/msg/Twist':stop['token']='fixture-token'
            assert command(ws,stop)['agv']['execution']=='published_not_hardware_acknowledged'
        pubs['fisheye'].publish(Image(height=8,width=8,encoding='mono8',step=8,data=[1]*64))
        time.sleep(.1);invalid=command(ws,{},'check')['fisheye']
        assert invalid['received']==invalid['accepted']+invalid['invalid'] and invalid['invalid']==1
        assert command(ws,{'camera':{'command':'stop_stream'}})['camera']['result']=='yes'
        pubs['fisheye'].publish(stamp(Image(height=8,width=8,encoding='rgb8',step=24,data=[90,20,10]*64),STAMP_BASE+400_000_000))
        deadline=time.monotonic()+.3
        while time.monotonic()<deadline:
            try:packet=json.loads(ws.recv(timeout=.05))
            except TimeoutError:continue
            assert 'video' not in packet['payload']
        assert command(ws,{'camera':{'command':'start_stream'}})['camera']['result']=='yes'
        resumed=recv_until(ws,lambda m:m['payload'].get('video',{}).get('camera_id')=='fisheye')
        assert int(resumed['payload']['video']['stamp_ns'])==STAMP_BASE+400_000_000
        ws.send('{invalid');assert command(ws,{'unknown':{'command':'move'}})['unknown']['reason']=='unsupported_command'
        # Invalid header must not pass as camera liveness.
        pubs['mindvision'].publish(Image(height=8,width=8,encoding='mono8',step=8,data=[1]*64))
        time.sleep(.1);assert command(ws,{},'check')['mindvision']['reason']=='invalid_header_stamp'
        time.sleep(1.6);assert command(ws,{},'check')['fisheye']['reason']=='stale'
        if control:
            move['agv']['parameters']['vx']=.1
            assert command(ws,move)['agv']['reason']=='chassis_not_verified'
        ws.close();ws=connect(f'ws://127.0.0.1:{port}',proxy=None,open_timeout=1)
        assert command(ws,{},'check')['fisheye']['result']=='no'
    finally:
        if ws is not None:ws.close()
        node.destroy_node();rclpy.shutdown();proc.send_signal(signal.SIGINT)
        try:code=proc.wait(timeout=3)
        except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=2);raise
        log.close();assert code==0,(tmp_path/'bridge.log').read_text()


def test_remote_client_reconnect_without_wan_and_bounded_preview_load(tmp_path):
    import threading
    from websockets.sync.server import serve
    connections=[];errors=[];packets=[];latest_states={};mutex=threading.Lock()
    def handler(ws):
        with mutex:
            number=len(connections);connections.append(number)
        try:
            ws.send(json.dumps({'header':{'robot_id':'tank_inspection_robot','msg_type':'check'},'payload':{}}))
            checked=recv_until(ws,lambda m:m['header']['msg_type']=='response')
            assert checked['payload']['emat']['status']=='unverified'
            if number==0:ws.close();return
            for message in ws:
                record=json.loads(message)
                with mutex:
                    # Latest-only test observation; never retain the JPEG bodies.
                    if 'devices' in record['payload']:
                        latest_states.update({k:{field:v[field] for field in ['received','accepted','invalid']}
                                              for k,v in record['payload']['devices'].items()
                                              if k in ['fisheye','depth_camera']})
                    if 'video' in record['payload']:
                        v=record['payload']['video'];packets.append({'camera':v['camera_id'],'stamp_ns':int(v['stamp_ns']),
                            'drops':v['preview_dropped'],'source_received':v['source_received'],
                            'source_accepted':v['source_accepted'],'at':time.monotonic()})
        except BaseException as e:errors.append(repr(e))
    with serve(handler,'127.0.0.1',0) as server:
        port=server.socket.getsockname()[1]
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        binary=Path(get_package_prefix('agv_bridge'))/'lib/agv_bridge/websocket_bridge_node'
        params={'robot_id':'tank_inspection_robot','server_uri':f'ws://127.0.0.1:{port}','local_server_port':0,
                'tank_inspection.fisheye_topic':'/load/fisheye','tank_inspection.depth_camera_topic':'/load/depth'}
        config=tmp_path/'params.yaml';config.write_text(yaml.safe_dump({'websocket_bridge_node':{'ros__parameters':params}}))
        log=(tmp_path/'remote.log').open('w');proc=subprocess.Popen([str(binary),'--ros-args','--params-file',str(config)],stdout=log,stderr=subprocess.STDOUT)
        rclpy.init();node=rclpy.create_node('bounded_preview_probe')
        pubs=[node.create_publisher(Image,'/load/'+name,qos_profile_sensor_data) for name in ['fisheye','depth']]
        try:
            deadline=time.monotonic()+8
            while time.monotonic()<deadline and (len(connections)<2 or not all(p.get_subscription_count() for p in pubs)):
                rclpy.spin_once(node,timeout_sec=.02)
            assert len(connections)>=2 and all(p.get_subscription_count() for p in pubs) and not errors
            image=np.zeros((384,512,3),np.uint8);image[:,:,1]=70;data=image.tobytes();rss=[]
            start=time.monotonic();cpu_start=Path(f'/proc/{proc.pid}/stat').read_text().split();cpu_start=int(cpu_start[13])+int(cpu_start[14])
            for i in range(80):
                for p in pubs:p.publish(stamp(Image(height=384,width=512,encoding='rgb8',step=1536,data=data),STAMP_BASE+i*50000000))
                until=start+(i+1)*.05
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
                if i in [39,79]:
                    fields=Path(f'/proc/{proc.pid}/status').read_text().splitlines();rss.append(int(next(x for x in fields if x.startswith('VmRSS:')).split()[1])*1024)
            cpu_end=Path(f'/proc/{proc.pid}/stat').read_text().split();cpu_ticks=int(cpu_end[13])+int(cpu_end[14])-cpu_start
            # Drain asynchronous publication/reporting after fixed20Hz input;
            # this does not lower playback rate or assert complete preview capture.
            drain_until=time.monotonic()+.3
            while time.monotonic()<drain_until:rclpy.spin_once(node,timeout_sec=.01)
            with mutex:seen=list(packets);counts=dict(latest_states)
            assert set(v['camera'] for v in seen)=={'fisheye','depth_camera'} and not errors
            assert rss[1]-rss[0]<16*1024*1024,(rss,seen)
            for camera in ['fisheye','depth_camera']:
                values=[v for v in seen if v['camera']==camera]
                assert len(values)<=22 and values[-1]['drops']>0
                assert all(values[i]['stamp_ns']<values[i+1]['stamp_ns'] for i in range(len(values)-1))
            balances={}
            for camera in ['fisheye','depth_camera']:
                values=[v for v in seen if v['camera']==camera];replaced=values[-1]['drops']
                state=counts[camera]
                assert state['received']==state['accepted']+state['invalid']
                # The periodic device status and video report are independent
                # messages.  Account against the counters snapshotted with the
                # final video, not a status that may be one timer cycle older.
                received=values[-1]['source_received'];accepted=values[-1]['source_accepted']
                invalid=received-accepted;pending=accepted-len(values)-replaced
                assert received>=accepted and accepted==len(values)+replaced
                assert pending>=0 and 0<received<=80
                balances[camera]={'input':80,'received':received,'accepted':accepted,'invalid':invalid,
                    'dds_unreceived':80-received,'preview_generated':len(values),'latest_replaced':replaced,'pending':pending}
                assert 80==balances[camera]['dds_unreceived']+invalid+len(values)+replaced+pending
            latency_ms=[(v['at']-start-(v['stamp_ns']-STAMP_BASE)/1e9)*1000 for v in seen]
            print(json.dumps({'wireless_preview_load':{'input_frames':160,'preview_messages':len(seen),'input_rate_hz_each':20.,'wall_sec':time.monotonic()-start,'cpu_sec':cpu_ticks/os.sysconf('SC_CLK_TCK'),'rss_at40_and80_frames':rss,'conservation_by_camera':balances,'preview_delivery_p50_ms':float(np.percentile(latency_ms,50)),'preview_delivery_p95_ms':float(np.percentile(latency_ms,95)),'physical_radio_tested':False,'comparison_baseline':'new feature; no prior comparable transport, no fabricated percent regression'}}))
        finally:
            node.destroy_node();rclpy.shutdown();proc.send_signal(signal.SIGINT)
            try:code=proc.wait(timeout=3)
            except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=2);raise
            log.close();server.shutdown();worker.join(2);assert code==0,(tmp_path/'remote.log').read_text()
        assert not errors,errors


def test_local_port_collision_fails_without_killing_existing_listener(tmp_path):
    binary=Path(get_package_prefix('agv_bridge'))/'lib/agv_bridge/websocket_bridge_node'
    with socket.socket() as listener:
        listener.bind(('0.0.0.0',0));listener.listen();port=listener.getsockname()[1]
        config=tmp_path/'params.yaml';config.write_text(yaml.safe_dump({'websocket_bridge_node':{'ros__parameters':{'robot_id':'tank_inspection_robot','server_uri':'','local_server_port':port}}}))
        result=subprocess.run([str(binary),'--ros-args','--params-file',str(config)],capture_output=True,text=True,timeout=4)
        assert result.returncode==2 and 'listen failed' in result.stderr
        assert listener.getsockname()[1]==port


@pytest.mark.parametrize('target,poll_ms', [(5., 200), (15., 20)])
def test_preview_deadline_rate_and_decodable_frames(tmp_path, target, poll_ms):
    """Fixed30Hz DDS input; distinguish display delivery from source/algorithm rate."""
    import threading
    with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    config=tmp_path/'params.yaml'
    params={'robot_id':'tank_inspection_robot','server_uri':'','local_server_port':port,
            'report_period_ms':poll_ms,'tank_inspection.video_fps':target,
            'tank_inspection.depth_camera_topic':'/preview_rate/image'}
    config.write_text(yaml.safe_dump({'websocket_bridge_node':{'ros__parameters':params}}))
    binary=Path(get_package_prefix('agv_bridge'))/'lib/agv_bridge/websocket_bridge_node'
    log=(tmp_path/'bridge.log').open('w');proc=subprocess.Popen([str(binary),'--ros-args','--params-file',str(config)],stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();node=rclpy.create_node('preview_rate_fixture')
    pub=node.create_publisher(Image,'/preview_rate/image',qos_profile_sensor_data)
    frames=[];errors=[];stop=threading.Event();worker=None
    try:
        deadline=time.monotonic()+4
        while not pub.get_subscription_count() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        assert pub.get_subscription_count()
        ws=None
        while time.monotonic()<deadline and ws is None:
            try:ws=connect(f'ws://127.0.0.1:{port}',proxy=None,open_timeout=.3)
            except (OSError,TimeoutError):time.sleep(.02)
        assert ws is not None,'DDS subscription discovery can precede WebSocket listen readiness'
        with ws:
            def receive():
                while not stop.is_set():
                    try:record=json.loads(ws.recv(timeout=.1))
                    except TimeoutError:continue
                    except Exception as exc:
                        if not stop.is_set():errors.append(repr(exc))
                        return
                    if 'video' not in record['payload']:continue
                    v=record['payload']['video'];decoded=cv2.imdecode(np.frombuffer(base64.b64decode(v['data']),np.uint8),cv2.IMREAD_COLOR)
                    if decoded is None:errors.append('jpeg_decode_failed');continue
                    frames.append({'at':time.monotonic(),'stamp':int(v['stamp_ns']),'target':v['preview_target_fps'],'encode_ms':v['encode_wall_ms']})
            worker=threading.Thread(target=receive);worker.start()
            image=np.zeros((480,848,3),np.uint8);image[:,:,0]=50;image[:,:,2]=190
            start=time.monotonic()
            input_frames=150
            for i in range(input_frames):
                pub.publish(stamp(Image(height=480,width=848,encoding='rgb8',step=2544,data=image.tobytes()),STAMP_BASE+i*33333333))
                until=start+(i+1)/30
                while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.001)
            time.sleep(.25);stop.set();worker.join(1)
            assert not worker.is_alive() and not errors
            fps=(len(frames)-1)/(frames[-1]['at']-frames[0]['at'])
            assert .9*target <= fps <= 1.1*target,(fps,frames)
            assert all(frames[i]['stamp']<frames[i+1]['stamp'] for i in range(len(frames)-1))
            assert all(v['target']==target for v in frames)
            print(json.dumps({'preview_deadline_rate':{'input_hz':30,'input_frames':input_frames,'target':target,'poll_ms':poll_ms,'decoded':len(frames),'decoded_fps':fps,'encode_p95_ms':float(np.percentile([v['encode_ms'] for v in frames],95)),'physical_radio_tested':False}}))
    finally:
        stop.set()
        if worker:worker.join(1)
        node.destroy_node();rclpy.shutdown();proc.send_signal(signal.SIGINT);code=proc.wait(timeout=4);log.close()
        assert code==0,(tmp_path/'bridge.log').read_text()
