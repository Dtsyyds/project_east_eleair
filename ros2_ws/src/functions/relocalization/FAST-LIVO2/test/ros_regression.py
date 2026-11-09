#!/usr/bin/env python3
"""Isolated synthetic ROS 2 regressions. Never starts a driver or touches project maps."""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import yaml

os.environ.setdefault("ROS_DOMAIN_ID", "179")
os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
os.environ.setdefault("ROS_LOG_DIR", "/tmp/ultra-regression-ros-logs")
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import Imu, Image, CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from fast_livo.msg import State6Stamped
from fast_livo.srv import SetInitialPose
from tf2_msgs.msg import TFMessage


class Runner:
    def __init__(self, install, temporary):
        self.install, self.temporary = install, temporary
        self.node = rclpy.create_node("ultra_regression_observer")
        self.processes = []
        self.results = []
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.imu = self.node.create_publisher(Imu, "/livox/imu", 10)
        self.lidar = self.node.create_publisher(CustomMsg, "/livox/lidar", 10)
        self.initial_pose_client = self.node.create_client(SetInitialPose, "/fast_livo2/set_initial_pose")
        self.cloud_points = []
        self.acceleration=(0.,0.,9.81)
        for a in range(20):
            for b in range(20):
                self.cloud_points.extend([(1.01+a*.02,1.01+b*.02,1.),
                                          (1.,1.01+a*.02,1.01+b*.02),
                                          (1.01+a*.02,1.,1.01+b*.02)])
        self.map = self.temporary / "prior.pcd"
        self.write_map(self.map, self.cloud_points)
        self.sparse = self.temporary / "sparse.pcd"
        self.write_map(self.sparse, [(1+2*i,1,1) for i in range(20)])

    def check(self, condition, label):
        if not condition:
            raise AssertionError(label)
        self.results.append(label)
        print("PASS", label, flush=True)

    def write_map(self, file, points):
        file.write_text("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH " + str(len(points)) +
                        "\nHEIGHT 1\nPOINTS " + str(len(points)) + "\nDATA ascii\n" +
                        "\n".join(" ".join(map(str, p)) for p in points)+"\n")

    def start(self, package, executable, parameters, *, executable_prefix=None, extra_arguments=()):
        config=self.temporary/f"params-{len(self.processes)}.yaml"
        config.write_text(yaml.safe_dump({"/**":{"ros__parameters":parameters}}))
        log=self.temporary/f"process-{len(self.processes)}.log"
        stream=log.open("w")
        prefix=self.install/package if executable_prefix is None else executable_prefix
        process=subprocess.Popen([str(prefix/"lib"/package/executable),"--ros-args","--params-file",str(config),*extra_arguments],
                                 stdout=stream,stderr=subprocess.STDOUT)
        self.processes.append((process,stream,log))
        return process,log

    def stop(self, process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()

    def spin(self, seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(self.node,timeout_sec=.01)

    def base(self):
        return {"time_check.enabled":False,"common.img_en":0,"pcd_save.pcd_save_en":False,
                "extrin_calib.extrinsic_R":[1.,0.,0.,0.,1.,0.,0.,0.,1.],
                "extrin_calib.extrinsic_T":[0.,0.,0.],"imu.imu_int_frame":2,
                "preprocess.point_filter_num":1,"preprocess.blind":.1,
                "preprocess.filter_size_surf":.02,"publish.pub_plane_en":False}

    def feed(self, seconds, shift=0.):
        end=time.monotonic()+seconds
        next_scan=0
        while time.monotonic()<end:
            now=self.node.get_clock().now()
            imu=Imu(); imu.header.stamp=now.to_msg()
            imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z=self.acceleration
            self.imu.publish(imu)
            if time.monotonic()>=next_scan:
                cloud=CustomMsg(); cloud.header.stamp=rclpy.time.Time(nanoseconds=now.nanoseconds-100_000_000).to_msg()
                cloud.header.frame_id="livox_frame"
                for i,(x,y,z) in enumerate(self.cloud_points):
                    p=CustomPoint();p.x=x+shift;p.y=y;p.z=z;p.tag=0x10;p.line=0;p.reflectivity=100
                    p.offset_time=int(i*99_000_000/max(1,len(self.cloud_points)-1));cloud.points.append(p)
                cloud.point_num=len(cloud.points);self.lidar.publish(cloud)
                next_scan=time.monotonic()+.1
            rclpy.spin_once(self.node,timeout_sec=.002)
            time.sleep(.003)

    def until(self, condition, timeout=3.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline and not condition():
            rclpy.spin_once(self.node,timeout_sec=.01)
        return condition()

    def set_pose(self, pose, request_id=None):
        assert self.initial_pose_client.wait_for_service(timeout_sec=3.),"Initial-pose service unavailable"
        request=SetInitialPose.Request(request_id=request_id if request_id is not None else str(time.monotonic_ns()),pose=pose)
        future=self.initial_pose_client.call_async(request)
        assert self.until(future.done),"Initial-pose service response timed out"
        return future.result()

    def registration_parameters(self):
        return {"prior_map_path":str(self.temporary),"prior_map_name":self.map.name,
                "extrin_calib.extrinsic_R":[1.,0.,0.,0.,1.,0.,0.,0.,1.],
                "extrin_calib.extrinsic_T":[.1,0.,0.],"algorithm":"icp","num_threads":2,
                "map_voxel_leaf_size":.04,"gicp_map_voxel_leaf_size":.04,
                "gicp_cloud_voxel_leaf_size":.04,"converged_count_thre":2,"gicp_solver_max_iter":20,
                "initial_pose_timeout_seconds":4.,"timeout_seconds":5.}

    def initial_pose_service(self):
        parameters=self.base()
        parameters.update(locate_in_prior_map=True,prior_map_path=str(self.temporary),prior_map_name=self.map.name)
        final=[]
        subscription=self.node.create_subscription(PoseWithCovarianceStamped,"/fast_livo2/initial_map_pose",final.append,self.qos)
        diagnostic=self.node.create_publisher(PoseWithCovarianceStamped,"/icp_result",self.qos)
        process,log=self.start("fast_livo","fastlivo_mapping",parameters)
        try:
            self.check(self.initial_pose_client.wait_for_service(timeout_sec=3.),"mapping exposes initial-pose service")
            pose=PoseWithCovarianceStamped();pose.header.frame_id="map";pose.pose.pose.orientation.w=2.
            diagnostic.publish(pose);self.spin(.2)
            self.check(not any(x.node_name=="laserMapping" for x in self.node.get_subscriptions_info_by_topic("/icp_result")),
                       "diagnostic topic cannot bypass service acceptance")
            for label,change in [
                ("wrong frame",lambda p:setattr(p.header,"frame_id","wrong")),
                ("nonfinite position",lambda p:setattr(p.pose.pose.position,"x",float("nan"))),
                ("zero quaternion",lambda p:setattr(p.pose.pose.orientation,"w",0.)),
                ("nonfinite covariance",lambda p:p.pose.covariance.__setitem__(0,float("inf")))]:
                invalid=copy.deepcopy(pose);change(invalid)
                response=self.set_pose(invalid)
                self.check(not response.accepted and bool(response.message),"service rejects "+label+" with reason")
            self.check(not self.set_pose(pose,"").accepted,"service rejects empty request ID")
            self.check(not any(x.node_name=="laserMapping" for x in self.node.get_subscriptions_info_by_topic("/livox/imu")),
                       "rejected service requests leave sensors unsubscribed")
            self.check(self.set_pose(pose,"accepted-request").accepted,"service accepts valid pose without waiting for sensor initialization")
            self.spin(.1)
            self.check(not final and process.poll() is None,"acceptance succeeds before prior-map initialization notification")
            sensors=[x for x in self.node.get_subscriptions_info_by_topic("/livox/imu") if x.node_name=="laserMapping"]
            self.check(len(sensors)==1,"accepted service request creates one IMU subscription")
            self.check(self.set_pose(pose,"accepted-request").accepted,"identical request retry returns accepted")
            altered=copy.deepcopy(pose);altered.pose.pose.position.x=9.
            self.check(not self.set_pose(altered,"accepted-request").accepted,"same request ID with changed pose is rejected")
            self.check(not self.set_pose(pose,"another-request").accepted,"another request cannot replace the initial pose")
            sensors=[x for x in self.node.get_subscriptions_info_by_topic("/livox/imu") if x.node_name=="laserMapping"]
            self.check(len(sensors)==1 and log.read_text().count("Initial IMU pose accepted;")==1,
                       "retries never repeat sensor initialization or acceptance")
        finally:
            self.stop(process)
            self.node.destroy_subscription(subscription);self.node.destroy_publisher(diagnostic)

    def registration_delivery(self):
        parameters=self.registration_parameters();parameters["initial_pose_timeout_seconds"]=12.
        diagnostic=[];final=[]
        subscriptions=[self.node.create_subscription(PoseWithCovarianceStamped,"/icp_result",diagnostic.append,self.qos),
                       self.node.create_subscription(PoseWithCovarianceStamped,"/fast_livo2/initial_map_pose",final.append,self.qos)]
        solver,solver_log=self.start("relocalization","teaser_gicp_node",parameters)
        mapper=None
        try:
            self.spin(.4);self.feed(.8)
            self.check(self.until(lambda:bool(diagnostic)),"registration produces diagnostic pose before service exists")
            self.spin(5.3)
            self.check(solver.poll() is None,"topic delivery and old five-second window do not count as acceptance")
            mapping_parameters=self.base()
            mapping_parameters.update(locate_in_prior_map=True,prior_map_path=str(self.temporary),prior_map_name=self.map.name)
            mapper,mapper_log=self.start("fast_livo","fastlivo_mapping",mapping_parameters)
            self.check(self.until(lambda:solver.poll() is not None),"late-started mapping receives service result and solver exits")
            self.check(solver.returncode==0 and "Mapping confirmed initial pose acceptance" in solver_log.read_text(),
                       "solver success requires explicit mapping acceptance")
            self.check(mapper.poll() is None and not final,"solver exits on acceptance without waiting for map initialization")
        finally:
            self.stop(solver)
            if mapper is not None:self.stop(mapper)
            for sub in subscriptions:self.node.destroy_subscription(sub)
        self.spin(.3)

        for scenario in ("reject","lost_reply","no_reply","missing_server"):
            requests=[];service=None
            if scenario!="missing_server":
                def respond(request,response):
                    requests.append(copy.deepcopy(request))
                    response.accepted=scenario!="reject"
                    response.message="controlled acceptance" if response.accepted else "controlled rejection"
                    return response
                service=self.node.create_service(SetInitialPose,"/fast_livo2/set_initial_pose",respond)
                if scenario in ("lost_reply","no_reply"):
                    send=service.send_response
                    replies=[]
                    def maybe_send(response,header):
                        replies.append(response)
                        if scenario=="lost_reply" and len(replies)>1:send(response,header)
                    service.send_response=maybe_send
            parameters=self.registration_parameters()
            parameters["initial_pose_timeout_seconds"]=4. if scenario=="lost_reply" else 2.6
            solver,log=self.start("relocalization","teaser_gicp_node",parameters)
            try:
                self.spin(.4);self.feed(.8)
                self.check(self.until(lambda:solver.poll() is not None,6.),scenario+" delivery has bounded completion")
                if scenario=="reject":
                    self.check(solver.returncode==1 and "controlled rejection" in log.read_text(),"mapping rejection fails solver with reason")
                elif scenario=="lost_reply":
                    self.check(solver.returncode==0 and len(requests)>=2,"lost response is retried successfully")
                    self.check(all(request==requests[0] for request in requests),"retry preserves application ID, timestamp and pose")
                else:
                    self.check(solver.returncode==1 and "acceptance timed out" in log.read_text(),scenario+" never reports successful delivery")
            finally:
                self.stop(solver)
                if service is not None:self.node.destroy_service(service)
                self.spin(.3)

    def mapping(self, prior=False, sparse=False, gravity=False):
        parameters=self.base()
        parameters["uav.gravity_align_en"]=gravity
        self.acceleration=(2.,0.,(9.81**2-4)**.5) if gravity else (0.,0.,9.81)
        parameters.update({"locate_in_prior_map":prior,"prior_map_path":str(self.temporary),
                           "prior_map_name":(self.sparse if sparse else self.map).name})
        odom=[];final=[];state=[];valid=[]
        subscriptions=[self.node.create_subscription(Odometry,"/aft_mapped_to_init",odom.append,10),
                       self.node.create_subscription(PoseWithCovarianceStamped,"/fast_livo2/initial_map_pose",final.append,self.qos),
                       self.node.create_subscription(State6Stamped,"/fast_livo2/state6_stamped",state.append,10),
                       self.node.create_subscription(Bool,"/fast_livo2/tracking_valid",valid.append,self.qos)]
        process,log=self.start("fast_livo","fastlivo_mapping",parameters)
        try:
            self.spin(.6)
            if prior:
                self.check(process.poll() is None,"prior waits without exiting")
                info=self.node.get_subscriptions_info_by_topic("/livox/imu")
                self.check(not any(x.node_name=="laserMapping" for x in info),"no IMU subscription before initial pose")
                pose=PoseWithCovarianceStamped();pose.header.frame_id="map";pose.pose.pose.orientation.w=1.
                self.check(self.set_pose(pose).accepted,"prior accepts initial pose through service");self.spin(.3)
            self.feed(2.)
            if sparse:
                self.spin(.2)
                self.check(process.poll()==1 and not final,"zero-plane prior exits without final pose")
                self.check("zero valid planes" in log.read_text(),"zero-plane failure explains filter remediation")
            else:
                self.check(process.poll() is None,"synthetic LIO remains alive")
                self.check(len(odom)>2 and len(state)>2,"synthetic LIO publishes pose and stamped state")
                self.check(state[-1].valid and state[-1].header.frame_id=="aft_mapped","stamped state carries validity and frame")
                gravity=state[-1].projected_gravity
                self.check(abs(gravity.x**2+gravity.y**2+gravity.z**2-1)<1e-5,"stamped state unit gravity convention")
                if prior:
                    self.check(len(final)==1,"prior publishes exactly one final alignment")
                    self.feed(1.,shift=100.)
                    self.spin(.1)
                    self.check(process.poll()==1 and valid and not valid[-1].data,"constraint loss marks invalid and stops localization")
        finally:
            self.stop(process)
            if not sparse and not prior:
                self.check(process.returncode==0,"ordinary LIO SIGINT exits cleanly")
            for sub in subscriptions:self.node.destroy_subscription(sub)

    def failures(self):
        cases=[({"preprocess.filter_size_surf":0.},"invalid surface filter"),
               ({"pcd_save.interval":0},"zero save interval"),
               ({"extrin_calib.extrinsic_T":[0.]},"short IMU extrinsics"),
               ({"locate_in_prior_map":True,"prior_map_path":"/missing","prior_map_name":"map.pcd"},"missing prior map")]
        for overrides,label in cases:
            parameters=self.base();parameters.update(overrides)
            process,log=self.start("fast_livo","fastlivo_mapping",parameters)
            process.wait(timeout=5)
            self.check(process.returncode==1,label+" exits with code 1")

    def registration(self):
        for executable in ["icp_node","small_gicp_node","teaser_node","sac_ia_gicp","teaser_gicp_node"]:
            parameters=self.registration_parameters();parameters.pop("algorithm")
            process,log=self.start("relocalization",executable,parameters)
            try:
                ready=self.until(lambda:"TeaserGicpNode Initialized" in log.read_text() or process.poll() is not None)
                self.check(ready and process.poll() is None,executable+" starts with a valid prior map")
            finally:self.stop(process)
            self.check(process.returncode==0,executable+" SIGINT exits cleanly")
        process,log=self.start("relocalization","teaser_gicp_node",{})
        process.wait(timeout=5)
        self.check(process.returncode==1 and "Prior localization requires" in log.read_text(),
                   "registration without a prior configuration fails instead of publishing placeholders")
        parameters=self.registration_parameters()
        received=[]
        def accept(request,response):
            received.append(request.pose);response.accepted=True;response.message="test accepted";return response
        service=self.node.create_service(SetInitialPose,"/fast_livo2/set_initial_pose",accept)
        process,log=self.start("relocalization","teaser_gicp_node",parameters)
        try:
            self.spin(.5);self.feed(.7)
            self.check(self.until(lambda:process.poll() is not None),"ICP solver exits after service acceptance")
            self.check(process.returncode==0 and any(abs(m.pose.pose.position.x+.1)<.02 for m in received),
                       "ICP service delivers converted initial IMU pose")
        finally:
            self.stop(process);self.node.destroy_service(service)
        parameters["timeout_seconds"]=.2
        process,log=self.start("relocalization","teaser_gicp_node",parameters)
        process.wait(timeout=5)
        self.check(process.returncode==1 and "timed out" in log.read_text(),"registration timeout returns code 1")

    def visual_start(self):
        parameters=self.base()
        parameters.update({"common.img_en":1,"camera.model":"Pinhole","camera.width":640,"camera.height":360,
                           "camera.fx":328.,"camera.fy":327.,"camera.cx":319.,"camera.cy":179.,
                           "extrin_calib.Rcl":[1.,0.,0.,0.,1.,0.,0.,0.,1.],
                           "extrin_calib.Pcl":[0.,0.,0.],
                           "vio.img_point_cov":1000.5,"vio.outlier_threshold":999.5})
        process,log=self.start("fast_livo","fastlivo_mapping",parameters)
        self.spin(.5)
        self.check(process.poll() is None,"visual startup accepts Rcl/Pcl and fractional noise")
        self.check(self.until(lambda:any(x.node_name=="laserMapping" for x in self.node.get_subscriptions_info_by_topic("/left_camera/image"))),"visual startup creates image subscription")
        self.stop(process)
        self.check(process.returncode==0,"visual startup SIGINT exits cleanly")

    def final_tf(self):
        received=[]
        subscription=self.node.create_subscription(TFMessage,"/tf_static",received.append,self.qos)
        publisher=self.node.create_publisher(PoseWithCovarianceStamped,"/fast_livo2/initial_map_pose",self.qos)
        process,log=self.start("relocalization","transform_publisher",{})
        self.spin(.3)
        self.check(not received,"TF does not publish before final mapping alignment")
        self.until(lambda:any(x.node_name=="transform_publisher_node" for x in
                              self.node.get_subscriptions_info_by_topic("/fast_livo2/initial_map_pose")))
        inputs=[x for x in self.node.get_subscriptions_info_by_topic("/fast_livo2/initial_map_pose")
                if x.node_name=="transform_publisher_node"]
        # RMW graph discovery may report history UNKNOWN/depth 0; verify the
        # advertised reliability/durability and actual retained delivery below.
        self.check(len(inputs)==1 and inputs[0].qos_profile.reliability==ReliabilityPolicy.RELIABLE and
                   inputs[0].qos_profile.durability==DurabilityPolicy.TRANSIENT_LOCAL,
                   "TF fixed input uses reliable transient-local QoS")
        self.check(not any(x.node_name=="transform_publisher_node" for x in self.node.get_subscriptions_info_by_topic("/icp_result")),
                   "TF never subscribes to the diagnostic registration result")
        pose=PoseWithCovarianceStamped();pose.header.frame_id="wrong";pose.pose.pose.orientation.w=2.
        publisher.publish(pose);self.spin(.2)
        self.check(not received,"TF rejects invalid alignment frame")
        pose.header.frame_id="map";pose.pose.pose.position.x=3.
        publisher.publish(pose);self.spin(.2)
        self.check(received and received[-1].transforms[0].transform.translation.x==3. and received[-1].transforms[0].transform.rotation.w==1.,"TF uses final alignment and normalizes quaternion")
        before=len(received);pose.pose.pose.position.x=9.;publisher.publish(pose);self.spin(.2)
        self.check(len(received)==before,"TF freezes the first valid alignment")
        late=[]
        late_subscription=self.node.create_subscription(TFMessage,"/tf_static",late.append,self.qos)
        self.check(self.until(lambda:bool(late)) and process.poll() is None and
                   late[-1].transforms[0].header.frame_id=="map" and
                   late[-1].transforms[0].child_frame_id=="camera_init" and
                   late[-1].transforms[0].transform.translation.x==3.,
                   "resident TF publisher serves the fixed transform to late subscribers")
        self.node.destroy_subscription(late_subscription)
        self.stop(process);self.spin(.2);received.clear()
        pose.pose.pose.position.x=5.;publisher.publish(pose);self.spin(.2)
        process,log=self.start("relocalization","transform_publisher",{})
        self.check(self.until(lambda:any(x.transform.translation.x==5. for m in received for x in m.transforms)),
                   "TF receives the mapping alignment published before its startup")
        self.stop(process);self.node.destroy_subscription(subscription);self.node.destroy_publisher(publisher)

    def compressed_transport(self):
        import cv2
        import numpy as np
        source=np.full((48,64,3),(40,120,200),dtype=np.uint8)
        cases=[("default_base","/ultra_camera/image","/ultra_camera/image",".jpg","jpeg"),
               ("separate_bases","/ultra_bag/image","/ultra_livo/image",".png","png")]
        for label,input_base,output,extension,codec in cases:
            received=[]
            subscription=self.node.create_subscription(Image,output,received.append,10)
            publisher=self.node.create_publisher(CompressedImage,input_base+"/compressed",10)
            process,log=self.start("image_transport","republish",{
                "in_transport":"compressed","out_transport":"raw"},
                executable_prefix=Path(get_package_prefix("image_transport")),
                extra_arguments=["-r","in/compressed:="+input_base+"/compressed","-r","out:="+output])
            try:
                encoded_ok,encoded=cv2.imencode(extension,source)
                if not encoded_ok:raise RuntimeError("Cannot encode test image")
                message=CompressedImage();message.header.stamp=self.node.get_clock().now().to_msg()
                message.header.frame_id="test_camera";message.format="bgr8; "+codec+" compressed bgr8"
                message.data=encoded.tobytes()
                deadline=time.monotonic()+5.
                while not received and time.monotonic()<deadline and process.poll() is None:
                    publisher.publish(message);self.spin(.1)
                self.check(received and (received[-1].width,received[-1].height,received[-1].encoding)==(64,48,"bgr8")
                           and received[-1].header==message.header,codec+" "+label+" decompresses to Image and preserves header")
                decoded=np.frombuffer(received[-1].data,dtype=np.uint8).reshape(source.shape)
                difference=np.abs(decoded.astype(float)-source.astype(float)).mean()
                self.check(difference<(2 if codec=="jpeg" else .01),codec+" "+label+" preserves expected pixels")
            finally:
                self.stop(process);self.node.destroy_publisher(publisher);self.node.destroy_subscription(subscription)

    def shutdown_stress(self, repetitions):
        for index in range(repetitions):
            process,log=self.start("fast_livo","fastlivo_mapping",self.base())
            deadline=time.monotonic()+3.
            while "PCD 保存已关闭" not in log.read_text() and time.monotonic()<deadline and process.poll() is None:
                self.spin(.01)
            self.spin(.03+(index%5)*.007)
            self.stop(process)
            self.check(process.returncode==0,f"SIGINT wait-set race stress {index+1}/{repetitions}")

    def close(self):
        for process,stream,log in self.processes:self.stop(process);stream.close()
        self.node.destroy_node()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install",type=Path,required=True)
    parser.add_argument("--shutdown-repeats",type=int,default=0)
    args=parser.parse_args()
    rclpy.init()
    with tempfile.TemporaryDirectory(prefix="ultra-ros-regression-") as directory:
        runner=Runner(args.install,Path(directory))
        try:
            runner.failures();runner.mapping();runner.mapping(prior=True)
            runner.mapping(gravity=True);runner.mapping(prior=True,gravity=True)
            runner.mapping(prior=True,sparse=True);runner.initial_pose_service();runner.registration();runner.registration_delivery()
            runner.visual_start();runner.final_tf()
            runner.compressed_transport();runner.shutdown_stress(args.shutdown_repeats)
            report={"passed":len(runner.results),"checks":runner.results}
            (Path(os.environ.get("ULTRA_TEST_REPORT","/tmp/ultra-ros-test-report.json"))).write_text(json.dumps(report,ensure_ascii=False,indent=2))
            print("PASS",len(runner.results),"ROS integration checks")
        except Exception:
            for _,_,log in runner.processes:
                print("LOG",log,"\n",log.read_text()[-6000:],flush=True)
            raise
        finally:runner.close();rclpy.shutdown()

if __name__=="__main__":main()
