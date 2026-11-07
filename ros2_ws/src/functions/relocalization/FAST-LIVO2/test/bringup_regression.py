#!/usr/bin/env python3
"""Verify combined launches without opening a hardware driver or writing project maps."""
import argparse
import os
from pathlib import Path
import signal
import re
import subprocess
import sys
import tempfile
import time
from unittest.mock import patch

os.environ.setdefault('ROS_DOMAIN_ID','183')
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
os.environ.setdefault('ROS_LOG_DIR','/tmp/ultra-bringup-regression-logs')
sys.dont_write_bytecode=True
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
from ros_regression import Runner
from prior_extension_ros_regression import cluster


class ParameterLoader(yaml.SafeLoader):
    pass

ParameterLoader.add_constructor('tag:yaml.org,2002:python/tuple',
                                lambda loader,node:loader.construct_sequence(node))


def inspect_launch(name,config,driver_config):
    records=[]
    def record(action,context):
        command=[perform_substitutions(context,item) for item in action.cmd]
        parameters={}
        for index,item in enumerate(command[:-1]):
            if item=='--params-file':
                for values in yaml.load(Path(command[index+1]).read_text(),Loader=ParameterLoader).values():
                    parameters.update(values.get('ros__parameters',{}))
        records.append((Path(command[0]).name,parameters))
    arguments={'config_file':str(config),'driver_config_file':str(driver_config)}
    if name=='relocate_mapping.launch.py':arguments['start_delay']='0.0'
    service=LaunchService(noninteractive=True)
    service.include_launch_description(LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('fast_livo'))/'launch'/name)),
                                 launch_arguments=arguments.items())]))
    with patch.object(ExecuteProcess,'execute',record):
        assert service.run()==0,'Launch inspection failed'
    nodes=dict(records)
    expected={'livox_ros_driver2_node','fastlivo_mapping','rviz2'}
    if name=='relocate_mapping.launch.py':expected|={'transform_publisher','teaser_gicp_node'}
    assert len(records)==len(expected) and set(nodes)==expected,nodes.keys()
    assert nodes['livox_ros_driver2_node']['frame_id']=='bringup_driver_frame'
    mapping=nodes['fastlivo_mapping']
    configured=yaml.safe_load(Path(config).read_text())['/**']['ros__parameters']
    expected_root=configured.get('workspace_root','auto')
    assert mapping.get('workspace_root','auto')==expected_root
    assert mapping['prior_map_name']=='prior.pcd' and mapping['locate_in_prior_map']==(name=='relocate_mapping.launch.py')
    assert mapping['prior_map_update_en'] is True
    if 'teaser_gicp_node' in nodes:
        assert nodes['teaser_gicp_node']['workspace_root']==expected_root
        assert nodes['teaser_gicp_node']['prior_map_path']==mapping['prior_map_path']
        assert nodes['teaser_gicp_node']['prior_map_name']==mapping['prior_map_name']
        assert 'locate_in_prior_map' not in nodes['teaser_gicp_node']
        assert 'uav.gravity_align_en' not in nodes['teaser_gicp_node']
        assert 'initial_pose_topic' not in nodes['transform_publisher']
        assert 'initial_pose_transient_local' not in nodes['transform_publisher']
    print('PASS',name,'default driver/RViz, isolated driver config, shared mapping config',flush=True)


def configuration(share,directory,prior):
    data=yaml.safe_load((share/'config/mapping_relocation.yaml').read_text())
    p=data['/**']['ros__parameters']
    p.update(locate_in_prior_map=prior,workspace_root=str(directory),prior_map_path='.',prior_map_name='prior.pcd')
    p['time_check']['enabled']=False;p['common']['img_en']=0
    p['pcd_save'].update(pcd_save_en=True,pcd_save_en_relocate=True,
                         output_dir='collected',output_dir_relocate='relocated')
    p['extrin_calib']['extrinsic_R']=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
    p['extrin_calib']['extrinsic_T']=[0.,0.,0.]
    p['preprocess'].update(point_filter_num=1,filter_size_surf=.02,filter_size_map=.02,blind=.1)
    p['imu']['imu_int_frame']=2;p['uav']['gravity_align_en']=False
    p['publish']['pub_plane_en']=False
    p['relocalization'].update(map_voxel_leaf_size=.04,gicp_map_voxel_leaf_size=.04,
                              gicp_cloud_voxel_leaf_size=.04,converged_count_thre=2,
                              gicp_solver_max_iter=20,num_threads=2,initial_pose_timeout_seconds=5.,timeout_seconds=30.)
    file=directory/('localization.yaml' if prior else 'mapping.yaml')
    file.write_text(yaml.safe_dump(data));return file



def reject_no_prior(config,directory):
    command=['ros2','launch','relocalization','relocalization.launch.py',
             'config_file:='+str(config),'start_delay:=0.0']
    result=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=10)
    (directory/'reject-no-prior.log').write_text(result.stdout)
    assert result.returncode!=0 and '重定位入口仅支持先验模式' in result.stdout,result.stdout
    assert 'process started with pid' not in result.stdout,result.stdout
    print('PASS relocalization.launch.py rejects no-prior configuration before starting nodes',flush=True)


def runtime_check(runner,name,config,prior):
    local=[];global_pose=[];initial=[];valid=[]
    subscriptions=[runner.node.create_subscription(Odometry,'/aft_mapped_to_init',local.append,10),
                   runner.node.create_subscription(Odometry,'/aft_mapped_in_map',global_pose.append,10),
                   runner.node.create_subscription(PoseWithCovarianceStamped,'/fast_livo2/initial_map_pose',initial.append,runner.qos),
                   runner.node.create_subscription(Bool,'/fast_livo2/tracking_valid',valid.append,runner.qos)]
    log=runner.temporary/(name+'.log')
    command=['ros2','launch','fast_livo',name,'config_file:='+str(config),'start_driver:=false','enable_rviz:=false']
    if prior:command+=['start_delay:=0.0','algorithm:=icp']
    with log.open('w') as stream:
        process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            runner.spin(.8)
            deadline=time.monotonic()+8
            while time.monotonic()<deadline and process.poll() is None:
                runner.feed(.3)
                if len(local)>2 and valid and valid[-1].data and (not prior or (initial and global_pose)):break
            assert process.poll() is None and len(local)>2 and valid and valid[-1].data,log.read_text()
            names=runner.node.get_node_names()
            assert 'livox_lidar_publisher' not in names and 'rviz2' not in names,names
            if prior:
                assert len(initial)==1 and global_pose and global_pose[-1].header.frame_id=='map',log.read_text()
                assert 'transform_publisher' in names,names
                assert runner.until(lambda:'Mapping confirmed initial pose acceptance' in log.read_text()),log.read_text()
            else:
                assert 'teaser_gicp_node' not in names and 'transform_publisher' not in names,names
            process.send_signal(signal.SIGINT);process.wait(timeout=8)
            assert process.returncode==0,log.read_text()
            output=runner.temporary/('relocated' if prior else 'collected')
            maps=list(output.glob('map_*.pcd'))
            assert len(maps)==1,log.read_text()
            with maps[0].open('rb') as saved:
                points=re.search(rb'POINTS\s+(\d+)',saved.read(4096))
            assert points and int(points.group(1))>0,'Exit did not save observations'
            print('PASS',name,'synthetic tracking'+('/initial pose/global pose' if prior else '')+' and one SIGINT saves map/exits cleanly',flush=True)
        finally:
            if process.poll() is None:os.killpg(process.pid,signal.SIGKILL);process.wait()
            for subscription in subscriptions:runner.node.destroy_subscription(subscription)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install',type=Path,required=True)
    parser.add_argument('--inspect-only',action='store_true')
    args=parser.parse_args()
    share=Path(get_package_share_directory('fast_livo'))
    with tempfile.TemporaryDirectory(prefix='ultra-bringup-regression-') as temporary:
        directory=Path(temporary)/'workspace with spaces'
        directory.mkdir()
        configs=[configuration(share,directory,prior) for prior in (False,True)]
        driver=yaml.safe_load((Path(get_package_share_directory('livox_ros_driver2'))/'config/msg_MID360.yaml').read_text())
        driver['driver_parameters']['frame_id']='bringup_driver_frame'
        driver_config=directory/'driver.yaml';driver_config.write_text(yaml.safe_dump(driver))
        names=['collect_prior.launch.py','relocate_mapping.launch.py']
        for name,config in zip(names,configs):inspect_launch(name,config,driver_config)
        for root in ('auto',None):
            for name,config in zip(names,configs):
                data=yaml.safe_load(config.read_text())
                parameters=data['/**']['ros__parameters']
                if root is None:parameters.pop('workspace_root')
                else:parameters['workspace_root']=root
                variant=directory/(name+('-missing.yaml' if root is None else '-auto.yaml'))
                variant.write_text(yaml.safe_dump(data))
                inspect_launch(name,variant,driver_config)
        reject_no_prior(configs[0],directory)
        if args.inspect_only:return
        rclpy.init();runner=Runner(args.install,directory)
        try:
            # Distinct planar voxels remain observable while extension updates
            # them; the former tiny corner straddled voxel boundaries.
            runner.cloud_points=cluster(0)
            runner.write_map(runner.map,runner.cloud_points)
            for name,config,prior in zip(names,configs,(False,True)):runtime_check(runner,name,config,prior)
        finally:runner.close();rclpy.shutdown()
    print('PASS combined-launch regression scenarios',flush=True)

if __name__=='__main__':main()
