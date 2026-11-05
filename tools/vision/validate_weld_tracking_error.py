#!/usr/bin/env python3
"""Localhost ROS contract test using synthetic results, never inference/hardware.

Run with deploy/env.sh, ROS_DOMAIN_ID=188 and ROS_LOCALHOST_ONLY=1.
The child uses the real passive adapter with only its pipeline replaced by fixtures.
"""
from array import array
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'packages/d405_seam_tracking/src'),
             str(ROOT/'ros2_ws/src/functions/perception/d405_seam_ros')]


def fixture_node():
    from d405_seam_ros import node
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy
    original_create=Node.create_publisher
    def checked_create(self,msg_type,topic,qos,*args,**kwargs):
        publisher=original_create(self,msg_type,topic,qos,*args,**kwargs)
        if topic=='/weld_tracking/error':
            # DDS discovery does not communicate history/depth on all RMWs.
            assert publisher.qos_profile.depth==1
            assert publisher.qos_profile.history==HistoryPolicy.KEEP_LAST
        return publisher
    Node.create_publisher=checked_create
    class FixturePipeline:
        visual=None
        def process(self,frame,diagnostic=False):
            case=int(frame.image[0,0,0])
            if case==8:raise ValueError('intentional_offline_fixture_failure')
            center,angle=(288,3) if case!=1 else (352,-3)
            if case==2:center,angle=320,0
            y=np.arange(168,456,3,dtype=float)
            x=center+math.tan(math.radians(angle))*(y-360)
            result=dict(source=frame.metadata(),valid_2d=True,predicted_only=False,
                        state='VISION_2D',coordinate_2d='original_image_pixel_centers',
                        visual={'valid':True},confidence=.95,metric_valid=False,frame_id_3d=None,
                        points_2d_y_left_right=np.column_stack((y,x-10,x+10)).tolist(),
                        timing_ms={'total':0.1})
            if case==3:result.update(state='LOST',valid_2d=False,points_2d_y_left_right=[],confidence=0.)
            if case==4:result.update(state='PARTIAL_2D')
            if case==5:result.update(state='PREDICTED',predicted_only=True)
            if case==6:result['points_2d_y_left_right']=[p for p in result['points_2d_y_left_right'] if p[0]<330]
            if case==10:result['points_2d_y_left_right']=[p for p in result['points_2d_y_left_right'] if 336<=p[0]<=351]
            if case==11:result['points_2d_y_left_right']=[p for p in result['points_2d_y_left_right'] if 336<=p[0]<=348]
            if case==12:result['points_2d_y_left_right']=[p for p in result['points_2d_y_left_right'] if p[0]<=378]
            if case==13:result['points_2d_y_left_right']=[p for p in result['points_2d_y_left_right'] if p[0] in (336,339,342,369,372,375)]
            return result,{}
        def close(self):pass
    node.create_pipeline=lambda _:FixturePipeline()
    node.main(['offline_error_fixture','--config',str(ROOT/'config/d405/round1_a.yaml'),
               '--stream','color','--depth-scale-m','.001'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture-node',action='store_true')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='188' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('requires ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1')
    if args.fixture_node:return fixture_node()
    if args.output is None:parser.error('--output required')
    args.output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy
    from sensor_msgs.msg import Image
    from duct_robot_msgs.msg import WeldTrackingError
    rclpy.init();fixture=Node('weld_error_contract_fixture')
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    received=[]
    fixture.create_subscription(WeldTrackingError,'/weld_tracking/error',received.append,qos)
    publisher=fixture.create_publisher(Image,'/d405/camera/color/image_raw',qos)
    process=None
    def wait(predicate,timeout=15):
        deadline=time.monotonic()+timeout
        while not predicate():
            if process is not None and process.poll() is not None:raise RuntimeError('fixture node exited')
            if time.monotonic()>deadline:raise TimeoutError('ROS contract fixture timed out')
            rclpy.spin_once(fixture,timeout_sec=.05)
    rows=[]
    try:
        with (args.output/'node.log').open('w') as log:
            process=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--fixture-node'],stdout=log,stderr=subprocess.STDOUT)
            wait(lambda:publisher.get_subscription_count()>0 and len(fixture.get_publishers_info_by_topic('/weld_tracking/error'))==1)
            info=fixture.get_publishers_info_by_topic('/weld_tracking/error')[0]
            assert info.topic_type=='duct_robot_msgs/msg/WeldTrackingError'
            assert info.qos_profile.reliability==ReliabilityPolicy.BEST_EFFORT
            assert info.qos_profile.durability==DurabilityPolicy.VOLATILE
            valid_cases=(0,1,2,9,10,12,13)
            for case in range(14):
                message=Image();message.header.frame_id='offline_fixture_color_optical_frame'
                message.header.stamp.sec=1793664000;message.header.stamp.nanosec=123456789+case*1000000
                message.width=320 if case==7 else 640;message.height=480;message.step=message.width*3;message.encoding='rgb8'
                pixels=np.zeros((message.height,message.width,3),np.uint8);pixels[0,0,0]=case
                message.data=array('B',pixels.tobytes());publisher.publish(message)
                wait(lambda:len(received)>=case+1)
                actual=received[case]
                assert actual.frame_seq==case+1 and actual.header==message.header
                assert actual.valid==(case in valid_cases)
                expected=(-.1,math.radians(3)) if case in (0,9,10,12,13) else ((.1,math.radians(-3)) if case==1 else (0.,0.))
                assert math.isclose(actual.lateral_error_norm,expected[0],abs_tol=1e-12)
                assert math.isclose(actual.heading_error_rad,expected[1],abs_tol=1e-12)
                assert math.isclose(actual.confidence,.95 if actual.valid else 0.,abs_tol=1e-6)
                rows.append(dict(case=case,seq=actual.frame_seq,valid=actual.valid,lateral=actual.lateral_error_norm,
                                 heading=actual.heading_error_rad,source_stamp_exact=True,source_frame_exact=True))
            end=time.monotonic()+.3
            while time.monotonic()<end:rclpy.spin_once(fixture,timeout_sec=.02)
            assert len(received)==14
    finally:
        if process is not None:
            process.send_signal(signal.SIGINT);process.wait(timeout=10)
        fixture.destroy_node();rclpy.shutdown()
    assert process.returncode==0
    report=dict(passed=True,frames=rows,qos='KeepLast(1)/BestEffort/Volatile',messages_per_processed_frame=1,
                inference_run=False,hardware_started=False,localhost_only=True,ros_domain_id=188)
    (args.output/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
