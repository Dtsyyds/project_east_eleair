"""CustomMsg DDS -> installed bridge -> actual R6 console in Firefox."""
import base64
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.qos import qos_profile_sensor_data

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('custom_browser_helpers', ROOT/'tests/system/test_websocket_console.py')
helpers = importlib.util.module_from_spec(spec); spec.loader.exec_module(helpers)
browser = helpers.browser


def test_custommsg_reaches_r6_console(browser, tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0')
    port = helpers.free_port()
    class Handler(SimpleHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def do_GET(self):
            if self.path.split('?')[0] == '/connection_config.js':
                body = ('window.RobotConnectionDefaults=' + json.dumps({
                    'unified': True, 'inspection': f'ws://127.0.0.1:{port}',
                    'robot': 'ws://127.0.0.1:9/ws/robot'}) + ';').encode()
                self.send_response(200); self.send_header('Content-Length', str(len(body)))
                self.end_headers(); self.wfile.write(body)
            else:
                super().do_GET()
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(Handler, directory=str(ROOT/'apps/web_console_chassis_tracking')))
    serving = threading.Thread(target=server.serve_forever, daemon=True); serving.start()
    binary = Path(get_package_prefix('agv_bridge'))/'lib/agv_bridge/websocket_bridge_node'
    with (tmp_path/'bridge.log').open('w') as log:
        process = subprocess.Popen([str(binary), '--ros-args', '-p', f'local_server_port:={port}',
            '-p', 'robot_id:=tank_inspection_robot', '-p', 'server_uri:=""',
            '-p', 'tank_inspection.lidar_topic:=/custom_browser/lidar'], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init(); node = rclpy.create_node('custom_browser_fixture')
    pub = node.create_publisher(CustomMsg, '/custom_browser/lidar', qos_profile_sensor_data)
    stop = threading.Event()
    def publish():
        count = 0
        while not stop.wait(.1):
            count += 1
            msg = CustomMsg(timebase=1800000000000000123+count, point_num=6001,
                            points=[CustomPoint(x=2*math.cos(i/100), y=2*math.sin(i/100), z=(i%30)/100.)
                                    for i in range(6001)])
            msg.header.frame_id = 'livox_frame'
            msg.header.stamp.sec = 1800000000
            msg.header.stamp.nanosec = count
            pub.publish(msg)
    publisher = threading.Thread(target=publish, daemon=True); publisher.start()
    try:
        browser.open(f'http://127.0.0.1:{server.server_port}/?ws=ws://127.0.0.1:{port}')
        browser.wait('window.LidarPreview && window.LidarPreview.snapshot().fresh', timeout=15)
        browser.wait('window.LidarPreview.snapshot().displayed>0 && window.LidarPreview.snapshot().vehicle_triangles>1000')
        state = browser.js('return window.LidarPreview.snapshot()')
        assert state['source_points'] == 6001 and 0 < state['points'] <= 2000
        assert state['vehicle_model_id'] == 'r6_full_assembly_20261107'
        assert state['frame_id'] == 'livox_frame'
        device = browser.js('return window.tankConsole.snapshot().devices.lidar')
        assert device['message_type'] == 'livox_ros_driver2/msg/CustomMsg'
        assert device['status'] == 'verified'
        (tmp_path/'live_custommsg.png').write_bytes(base64.b64decode(browser.call('WebDriver:TakeScreenshot', {})))
        (tmp_path/'live_custommsg.json').write_text(json.dumps({'renderer': state, 'device': device}, indent=2))
        stop.set(); publisher.join(timeout=3)
        browser.wait('!window.LidarPreview.snapshot().fresh', timeout=6)
    finally:
        stop.set(); publisher.join(timeout=3)
        node.destroy_node(); rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
        server.shutdown(); server.server_close(); serving.join(timeout=2)
