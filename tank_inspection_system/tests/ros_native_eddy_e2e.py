"""Installed C++ sync node + real ROS types/QoS; synthetic messages only."""
from pathlib import Path
import os,json,subprocess,time,sys,uuid
os.environ['ROS_DOMAIN_ID']='186'
os.environ['ROS_LOCALHOST_ONLY']='1'
ROOT=Path(__file__).resolve().parents[1]
import argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path)
args=parser.parse_args()
RESULT=args.output or ROOT/'reports/refactor_v1/ros_native_eddy_e2e.json'
if args.output and RESULT.exists(): parser.error('refusing to overwrite an existing result')
REPORT=RESULT.parent
REPORT.mkdir(parents=True,exist_ok=True)
os.environ['ROS_LOG_DIR']=str(REPORT/'ros_logs')
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, Imu
from nav_msgs.msg import Odometry
from eddy_driver.msg import EddyFrame, EddyChannelMean
from inspection_interfaces.msg import FusionIndex

def stamp(header,ns):
    header.stamp.sec,header.stamp.nanosec=divmod(ns,1000000000)
    header.frame_id='fixture'

def run(topic):
    sync_name='inspection_sync_fixture_'+uuid.uuid4().hex[:12]
    process=subprocess.Popen([str(ROOT/'install_cpp/inspection_sync/lib/inspection_sync/inspection_sync_node'),
                             '--ros-args','-r','__node:='+sync_name,'-p','anchor_topic:='+topic],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    node=rclpy.create_node('native_eddy_probe')
    pubs={}
    for name,cls in [('/camera/front/image_raw',Image),('/livox/lidar',PointCloud2),('/livox/imu',Imu),('/robot/odometry',Odometry),(topic,EddyFrame if topic=='/eddy/data' else EddyChannelMean)]:
        pubs[name]=node.create_publisher(cls,name,qos_profile_sensor_data)
    received=[]
    anchor_wall_times=[]
    fusion_sub=node.create_subscription(FusionIndex,'/derived/inspection/fusion_index',received.append,100)
    def matched():
        return (all(p.get_subscription_count() for p in pubs.values())
                and all(any(info.node_name==sync_name for info in node.get_subscriptions_info_by_topic(name)) for name in pubs)
                and node.count_publishers('/derived/inspection/fusion_index') > 0
                and any(info.node_name==sync_name for info in node.get_publishers_info_by_topic('/derived/inspection/fusion_index')))
    try:
        deadline=time.monotonic()+6
        while time.monotonic()<deadline and not matched():
            rclpy.spin_once(node,timeout_sec=.05)
        assert process.poll() is None, 'installed synchronizer exited early'
        assert matched(), 'DDS type/QoS matching with this process failed'
        # Discovery settles before any input is played. Per-frame pacing remains
        # the original .05s support lead plus .25s drain; no reduced-rate retry.
        until=time.monotonic()+.5
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        bases=[]
        for index in range(5):
            ns=10000000000+index*200000000; bases.append(ns)
            image=Image(height=1,width=1,encoding='mono8',step=1,data=[index]); stamp(image.header,ns)
            pubs['/camera/front/image_raw'].publish(image)
            cloud=PointCloud2(height=1,width=0);stamp(cloud.header,ns);pubs['/livox/lidar'].publish(cloud)
            for offset in [-10000000,10000000]:
                imu=Imu();stamp(imu.header,ns+offset);pubs['/livox/imu'].publish(imu)
                odom=Odometry();stamp(odom.header,ns+offset);odom.pose.pose.orientation.w=1.
                odom.pose.pose.position.x=index+offset/100000000.
                pubs['/robot/odometry'].publish(odom)
            # Wait for the supporting streams before publishing the anchor.
            until=time.monotonic()+.05
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
            if topic=='/eddy/data':
                msg=EddyFrame(sequence=index,samples_per_channel=20,channel_count=8,data=list(range(160)))
            else:msg=EddyChannelMean(channel_mean=[float(index)]*8)
            stamp(msg.header,ns);anchor_wall_times.append(time.monotonic());pubs[topic].publish(msg)
            until=time.monotonic()+.25
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        deadline=time.monotonic()+2
        while len(received)<5 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        assert len(received)==5, f'FusionIndex count {len(received)} != 5'
        assert [m.header.stamp.sec*1000000000+m.header.stamp.nanosec for m in received]==bases
        assert [m.anchor_sequence for m in received]==list(range(5))
        assert all(m.anchor_topic==topic and m.camera_valid and m.lidar_valid and m.imu_valid and m.pose_valid for m in received)
        assert all(m.quality_flags==0 and not m.asset_coordinate_valid for m in received)
        return {'topic':topic,'input_anchors':5,'output_fusion_indices':len(received),'type_qos_matched':True,
                'header_fields_valid':True,'quality_flags_zero':True,'asset_coordinates_valid':False,
                'nominal_header_rate_hz':5.,'anchor_pacing_seconds':.3,
                'measured_wall_rate_hz':(len(anchor_wall_times)-1)/(anchor_wall_times[-1]-anchor_wall_times[0]),
                'discovery_settle_seconds':.5,'sync_exit_code':0,'hardware_executed':False}
    finally:
        # Preserve actual fields/counts even when assertions fail.
        details=[{'stamp_ns':m.header.stamp.sec*1000000000+m.header.stamp.nanosec,
                  'anchor_sequence':m.anchor_sequence,'anchor_topic':m.anchor_topic,
                  'camera_valid':m.camera_valid,'lidar_valid':m.lidar_valid,
                  'imu_valid':m.imu_valid,'pose_valid':m.pose_valid,
                  'quality_flags':m.quality_flags,'asset_coordinate_valid':m.asset_coordinate_valid}
                 for m in received]
        (REPORT/('fields_'+topic.replace('/','_')+'.json')).write_text(json.dumps({
            'topic':topic,'installed_node':sync_name,'published_anchors':len(anchor_wall_times),
            'anchor_wall_times':anchor_wall_times,'received':details,'hardware_executed':False},indent=2)+'\n')
        node.destroy_node()
        process.send_signal(2)
        try:output=process.communicate(timeout=3)[0]
        except subprocess.TimeoutExpired:
            process.terminate();output=process.communicate(timeout=3)[0]
        (REPORT/('sync_'+topic.replace('/','_')+'.log')).write_bytes(output)
        assert process.returncode==0, f'sync exit code={process.returncode}'

result={'status':'environment_blocked','hardware_executed':False}
try:
    rclpy.init()
    result={'status':'passed','tests':[run('/eddy/data'),run('/eddy/channel_mean')],'hardware_executed':False}
except BaseException as exc:
    result.update({'status':'failed','error':str(exc)})
    raise
finally:
    if rclpy.ok():rclpy.shutdown()
    RESULT.write_text(json.dumps(result,indent=2)+'\n')
