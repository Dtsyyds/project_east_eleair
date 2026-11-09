#!/usr/bin/env python3
"""Check both nodes' resolved root parameters without sensors or project-map writes."""
import argparse
import os
from pathlib import Path
import sys
import tempfile

os.environ.setdefault('ROS_DOMAIN_ID', '185')
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
os.environ.setdefault('ROS_LOG_DIR', '/tmp/ultra-workspace-root-ros-logs')
os.environ.setdefault('PYTHONDONTWRITEBYTECODE', '1')
sys.dont_write_bytecode=True

import rclpy
from rcl_interfaces.srv import GetParameters
from ros_regression import Runner


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install',type=Path,required=True)
    args=parser.parse_args()
    expected_source=Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='ultra-workspace-root-ros-') as temporary:
        directory=Path(temporary)
        rclpy.init()
        runner=Runner(args.install,directory)
        try:
            manual_root=directory/'manual root with spaces'
            manual_root.mkdir()
            for package,executable,node_name in (
                    ('fast_livo','fastlivo_mapping','laserMapping'),
                    ('relocalization','teaser_gicp_node','teaser_gicp_node')):
                for configured in (None,'auto',str(manual_root)):
                    parameters=runner.base() if package=='fast_livo' else runner.registration_parameters()
                    if configured is not None:parameters['workspace_root']=configured
                    process,log=runner.start(package,executable,parameters)
                    client=runner.node.create_client(GetParameters,f'/{node_name}/get_parameters')
                    try:
                        assert client.wait_for_service(timeout_sec=4.),log.read_text()
                        future=client.call_async(GetParameters.Request(names=['workspace_root']))
                        assert runner.until(future.done),log.read_text()
                        expected=str(expected_source if configured in (None,'auto') else manual_root)
                        assert future.result().values[0].string_value==expected,log.read_text()
                        assert process.poll() is None,log.read_text()
                        assert f'workspace_root ({"manual" if configured==str(manual_root) else "auto"}): {expected}' in log.read_text()
                        print('PASS',package,'workspace_root',repr(configured),'->',expected,flush=True)
                    finally:
                        runner.stop(process)
                        runner.node.destroy_client(client)
                for invalid in ('','relative/workspace','~/Desktop/FAST_LIVO2_Ultra','AUTO'):
                    parameters=runner.base() if package=='fast_livo' else runner.registration_parameters()
                    parameters['workspace_root']=invalid
                    process,log=runner.start(package,executable,parameters)
                    assert process.wait(timeout=5.)!=0,log.read_text()
                    assert "workspace_root must be 'auto' or a non-empty absolute directory path" in log.read_text(),log.read_text()
                    print('PASS',package,'rejects',repr(invalid),flush=True)
        finally:
            runner.close()
            rclpy.shutdown()
    print('PASS 14 workspace-root node scenarios',flush=True)


if __name__=='__main__':main()
