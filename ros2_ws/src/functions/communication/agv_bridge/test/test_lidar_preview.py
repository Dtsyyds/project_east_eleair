"""Isolated DDS -> installed C++ bridge -> WebSocket point-cloud contract."""
import json
import os
from pathlib import Path
import signal
import socket
import copy
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.serialization import serialize_message
from ament_index_python.packages import get_package_prefix
from websockets.sync.client import connect


def test_custommsg_preview_finite_bound_invalid_stamps_and_stale(tmp_path):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
    binary = Path(get_package_prefix('agv_bridge'))/'lib/agv_bridge/websocket_bridge_node'
    command = [str(binary), '--ros-args', '-p', f'local_server_port:={port}',
               '-p', 'robot_id:=tank_inspection_robot', '-p', 'server_uri:=""',
               '-p', 'tank_inspection.lidar_topic:=/preview_test/lidar',
               '-p', 'tank_inspection.device_timeout_sec:=0.6']
    with (tmp_path/'bridge.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   env=dict(os.environ, ROS_LOG_DIR=str(tmp_path/'logs')))
    rclpy.init(); node = rclpy.create_node('lidar_preview_fixture')
    pub = node.create_publisher(CustomMsg, '/preview_test/lidar', qos_profile_sensor_data)
    ws = None
    try:
        deadline = time.monotonic()+8
        while time.monotonic()<deadline:
            try:
                ws = connect(f'ws://127.0.0.1:{port}', proxy=None, open_timeout=.3); break
            except (OSError, TimeoutError): time.sleep(.1)
        assert ws is not None, (tmp_path/'bridge.log').read_text()
        while not pub.get_subscription_count() and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert pub.get_subscription_count()
        time.sleep(.2)
        def preview(message):
            pub.publish(message)
            deadline = time.monotonic()+3
            while time.monotonic()<deadline:
                try: packet=json.loads(ws.recv(timeout=.3))
                except TimeoutError: continue
                cloud=packet['payload'].get('lidar_cloud')
                if cloud and cloud['stamp_ns']==str(message.header.stamp.sec*10**9): return cloud
            raise AssertionError('no point-cloud preview')
        msg=CustomMsg(timebase=1800000000000000123, point_num=4,
                      points=[CustomPoint(x=x,y=y,z=z,reflectivity=15,tag=16,line=2,offset_time=i*1000)
                              for i,(x,y,z) in enumerate([(1.,2.,3.),(-1.,-2.,-3.),
                                                        (float('nan'),0.,0.),(4.,5.,6.)])])
        msg.header.frame_id='test_lidar';msg.header.stamp.sec=100
        original=serialize_message(msg)
        cloud=preview(msg)
        assert cloud['points']==[[1.,2.,3.],[-1.,-2.,-3.],[4.,5.,6.]]
        assert cloud['source_points']==4 and cloud['preview_points']==3 and cloud['position_unit']=='m'
        assert cloud['frame_id']=='test_lidar' and cloud['timebase']==str(msg.timebase)
        assert serialize_message(msg)==original
        msg.points=[CustomPoint(x=float(i),y=2.,z=3.) for i in range(5001)]
        msg.point_num=5001;msg.header.stamp.sec=101
        cloud=preview(msg);assert len(cloud['points'])==1667 and cloud['sample_stride']==3
        assert cloud['points'][-1]==[4998.,2.,3.]

        def state_after(message, reason):
            pub.publish(message)
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                try: packet=json.loads(ws.recv(timeout=.2))
                except TimeoutError: continue
                assert 'lidar_cloud' not in packet['payload']
                state=packet['payload'].get('devices',{}).get('lidar',{})
                if state.get('reason')==reason:
                    assert state['status']=='failed'
                    assert state['message_type']=='livox_ros_driver2/msg/CustomMsg'
                    return
            raise AssertionError('missing failure: '+reason)

        state_after(msg,'repeated_header_stamp')
        invalid=copy.deepcopy(msg);invalid.header.stamp.sec=100
        state_after(invalid,'regressed_header_stamp')
        for sec,field,value in [(102,'point_num',0),(103,'point_num',5000),(104,'timebase',0)]:
            invalid=copy.deepcopy(msg);invalid.header.stamp.sec=sec;setattr(invalid,field,value)
            state_after(invalid,'invalid_livox_custom_msg')
        invalid=copy.deepcopy(msg);invalid.header.stamp.sec=0
        state_after(invalid,'invalid_header_stamp')
        msg.header.stamp.sec=105;preview(msg)
        # A valid recovery becomes stale after publication stops, with no cached resend.
        deadline=time.monotonic()+1.5;stale=False
        while time.monotonic()<deadline:
            try: packet=json.loads(ws.recv(timeout=.2))
            except TimeoutError: continue
            assert 'lidar_cloud' not in packet['payload']
            state=packet['payload'].get('devices',{}).get('lidar',{})
            stale=stale or state.get('reason')=='stale'
        assert stale
        assert process.poll() is None
    finally:
        if ws: ws.close()
        node.destroy_node();rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.kill();process.wait()
