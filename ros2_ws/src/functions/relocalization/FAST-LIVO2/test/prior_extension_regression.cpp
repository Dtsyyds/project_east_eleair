// Exercise the real initialization, matching, insertion and saving paths with
// overlapping scans. No driver, project PCD or physical sensor is used.
#include "LIVMapper.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <unistd.h>

namespace {
int checks = 0;
void check(bool condition, const std::string &label) {
  if (!condition) throw std::runtime_error(label);
  ++checks;
  std::cout << "PASS " << label << std::endl;
}

std::string bytes(const std::filesystem::path &file) {
  std::ifstream input(file, std::ios::binary);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

// Each cluster contains three orthogonal planar patches in separate root
// voxels. Consecutive scans share one cluster; scan 2 has no original prior.
PointCloudXYZI cluster(int index) {
  PointCloudXYZI cloud;
  for (int a = 0; a < 20; ++a) for (int b = 0; b < 20; ++b) {
    const double x = 1.1 + 2.0 * index;
    for (const V3D p : {V3D(x + a*.015, 1.1 + b*.015, 1.1),
                         V3D(x+.6, 1.1 + a*.015, 1.1 + b*.015),
                         V3D(x + a*.015, 1.7, 1.1 + b*.015)}) {
      PointType point{};
      point.x=p.x(); point.y=p.y(); point.z=p.z(); point.intensity=10+index;
      cloud.push_back(point);
    }
  }
  return cloud;
}

struct Fixture {
  std::filesystem::path directory, input;
  std::string input_bytes;
  std::unique_ptr<LIVMapper> mapper;
  rclcpp::Node::SharedPtr node;
  Eigen::Matrix4d alignment = Eigen::Matrix4d::Identity();
  int frame = 0;

  Fixture(const std::filesystem::path &root, const std::string &name,
          bool prior, std::optional<bool> extend, bool sliding=false)
      : directory(root/name), input(directory/"prior.pcd") {
    std::filesystem::create_directories(directory);
    alignment.block<3,3>(0,0)=Eigen::AngleAxisd(.35, V3D::UnitZ()).toRotationMatrix();
    alignment.block<3,1>(0,3)=V3D(5,-2,.7);
    PointCloudXYZI source;
    pcl::transformPointCloud(cluster(0), source, alignment.cast<float>().eval());
    if (pcl::io::savePCDFileBinary(input.string(), source)!=0) throw std::runtime_error("Cannot create fixture prior");
    input_bytes=bytes(input);
    std::vector<rclcpp::Parameter> params={
      {"time_check.enabled",false}, {"common.img_en",0}, {"locate_in_prior_map",prior},
      {"prior_map_path",directory.string()}, {"prior_map_name","prior.pcd"},
      {"extrin_calib.extrinsic_R",std::vector<double>{1,0,0,0,1,0,0,0,1}},
      {"extrin_calib.extrinsic_T",std::vector<double>{0,0,0}},
      {"uav.gravity_align_en",false}, {"publish.pub_plane_en",false}, {"publish.dense_map_en",true},
      {"preprocess.filter_size_surf",.01}, {"preprocess.filter_size_map",.01},
      {"lio.voxel_size",.5}, {"lio.min_eigen_value",.0025}, {"lio.max_layer",2},
      {"lio.beam_err",.05}, {"lio.dept_err",.02}, {"lio.sigma_num",3.},
      {"local_map.map_sliding_en",sliding}, {"local_map.half_map_size",4}, {"local_map.sliding_thresh",.1},
      {"pcd_save.pcd_save_en",true}, {"pcd_save.pcd_save_en_relocate",true},
      {"pcd_save.output_dir",(directory/"output").string()},
      {"pcd_save.output_dir_relocate",(directory/"output").string()}};
    if (extend) params.emplace_back("prior_map_update_en",*extend);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides(params);
    mapper=std::make_unique<LIVMapper>(node,name,options);
    mapper->initializeSubscribersAndPublishers(node);
    mapper->p_imu->unbiased_gyr.setZero();
    if (prior) {
      auto request=std::make_shared<fast_livo::srv::SetInitialPose::Request>();
      auto response=std::make_shared<fast_livo::srv::SetInitialPose::Response>();
      request->request_id=name;
      request->pose.header.frame_id="map";
      auto &p=request->pose.pose.pose;
      p.position.x=5; p.position.y=-2; p.position.z=.7;
      Eigen::Quaterniond q(alignment.block<3,3>(0,0));
      p.orientation.x=q.x(); p.orientation.y=q.y(); p.orientation.z=q.z(); p.orientation.w=q.w();
      mapper->set_initial_pose_cbk(request,response);
      check(response->accepted,name+" initial service accepts nonidentity map pose");
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id="map"; transform.child_frame_id="camera_init";
      transform.transform.translation.x=5; transform.transform.translation.y=-2; transform.transform.translation.z=.7;
      transform.transform.rotation=p.orientation;
      mapper->tf_buffer_->setTransform(transform,"extension_regression",true);
    }
    scan({0},0.);
    check(mapper->lidar_map_inited,name+" builds the initial voxel map");
    check(mapper->prior_map_update_en==extend.value_or(false),name+" reads the startup flag/default");
    check(mapper->voxelmap_manager->config_setting_.map_sliding_en==(sliding && (!prior || extend.value_or(false))),
          name+" applies the correct sliding policy");
  }

  void scan(std::initializer_list<int> clusters, double position) {
    auto &m=*mapper;
    m._state=StatesGroup(); m._state.pos_end=V3D(position,0,0);
    m.state_propagat=m._state;
    m.voxelmap_manager->state_=m._state;
    m.feats_undistort->clear();
    for (int index : clusters) for (auto point : cluster(index)) {
      point.x-=position;
      m.feats_undistort->push_back(point);
    }
    m.voxelmap_manager->feats_undistort_=m.feats_undistort;
    m.LidarMeasures.last_lio_update_time=100.+.1*frame++;
    m.handleLIO();
  }

  void checkAlignment(double position) {
    if (!mapper->locate_in_prior_map) return;
    check((mapper->initial_map_transform_-alignment).norm()<1e-9,"extension keeps the initial map transform fixed");
    const V3D expected=alignment.block<3,3>(0,0)*V3D(position,0,0)+alignment.block<3,1>(0,3);
    const auto &actual=mapper->odomAftMappedInMap.pose.pose.position;
    check((V3D(actual.x,actual.y,actual.z)-expected).norm()<.02,"global odometry remains in the original map coordinates");
  }

  std::string failThroughRun(bool fail_save=false, int next_cluster=100, double position=0.) {
    if (fail_save) {
      const auto blocked=directory/"not_a_directory";
      std::ofstream(blocked)<<"block output";
      mapper->pcd_output_dir=blocked.string();
    }
    auto timer=mapper->node->create_wall_timer(std::chrono::milliseconds(1),[this,next_cluster,position] {
      for (int i=0;i<mapper->max_unconstrained_frames;++i) scan({next_cluster,next_cluster+1},position);
    });
    try { mapper->run(node); }
    catch (const std::exception &error) { return error.what(); }
    throw std::runtime_error("Constraint loss did not escape run()");
  }
};

void extension(const std::filesystem::path &root, bool sliding) {
  Fixture f(root,sliding ? "extension_sliding" : "extension_full",true,true,sliding);
  auto &m=*f.mapper;
  auto &manager=*m.voxelmap_manager;
  const auto original=manager.voxel_map_;
  for (int step=0;step<6;++step) {
    const double position=1.5+2*step;
    f.scan({step,step+1},position);
    check(m.tracking_valid,"overlapping scan tracks, including scans with no original prior geometry");
    check(manager.effct_feat_num_>=manager.config_setting_.min_effective_points_,"new geometry supplies valid constraints");
    f.checkAlignment(position);
  }
  if (sliding) {
    bool old_removed=false;
    for (const auto &entry:original) old_removed|=manager.voxel_map_.count(entry.first)==0;
    check(old_removed,"sliding extension prunes distant prior voxels");
  } else {
    check(manager.voxel_map_.size()>original.size(),"extension inserts new root voxels");
    check(manager.countValidPlanes()>3,"extension fits planes beyond the original prior");
  }
  const auto cached=m.pcl_wait_save_intensity->size();
  check(cached==6*cluster(0).size()*2,"all successful observations are cached even when online voxels slide");
  const auto voxels=manager.voxel_map_.size();
  const auto poses=m.frame_num;
  const auto message=f.failThroughRun();
  check(message.find("lost constraints")!=std::string::npos && message.find("prior_extension")!=std::string::npos,
        "run preserves the original constraint-loss exception and mode");
  check(!m.tracking_valid && m.frame_num==poses,"failed scans do not publish valid poses");
  check(manager.voxel_map_.size()==voxels && m.pcl_wait_save_intensity->size()==cached,
        "failed scans do not alter online voxels or accepted observation cache");
  const auto saved=std::filesystem::path(m.pcd_output_dir)/(m.pcd_file_prefix+".pcd");
  PointCloudXYZI cloud;
  check(pcl::io::loadPCDFile(saved.string(),cloud)==0 && cloud.size()==cached,"caught tracking failure saves all and only accepted observations");
  bool transformed=false;
  const V3D expected=(f.alignment*Eigen::Vector4d(13.1,1.1,1.1,1.)).head<3>();
  for (const auto &p:cloud) transformed|=(V3D(p.x,p.y,p.z)-expected).norm()<.01;
  check(transformed,"saved unknown-region points use the original map frame");
  const auto metadata=bytes(std::filesystem::path(m.pcd_output_dir)/(m.pcd_file_prefix+"_metadata.yaml"));
  check(metadata.find("prior_map_update_en: true")!=std::string::npos,"metadata records the extension flag");
  check(bytes(f.input)==f.input_bytes,"source prior PCD remains byte-for-byte unchanged");
}
} // namespace

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  const auto root=std::filesystem::temp_directory_path()/("ultra-extension-"+std::to_string(getpid()));
  try {
    for (const std::optional<bool> flag : {std::optional<bool>{},std::optional<bool>{false}}) {
      Fixture f(root,flag ? "fixed_explicit" : "fixed_default",true,flag,true);
      auto &m=*f.mapper;
      const auto count=m.voxelmap_manager->voxel_map_.size();
      f.scan({0,1},1.5);
      check(m.tracking_valid && m.voxelmap_manager->voxel_map_.size()==count,"fixed prior accepts an overlapping scan without inserting unknown geometry");
      const auto cached=m.pcl_wait_save_intensity->size();
      const auto error=f.failThroughRun(false,1,3.5);
      check(error.find("prior_localization")!=std::string::npos && m.unconstrained_frames==5,"fixed mode retains five-frame loss protection");
      check(m.voxelmap_manager->voxel_map_.size()==count && m.pcl_wait_save_intensity->size()==cached,"fixed-mode failure cannot pollute map or cache");
      check(bytes(f.input)==f.input_bytes,"fixed-mode source prior is unchanged");
    }
    for (bool flag : {false,true}) {
      Fixture f(root,flag ? "mapping_flag_on" : "mapping_flag_off",false,flag,true);
      f.scan({0,1},1.5);
      f.scan({1,2},3.5);
      check(f.mapper->tracking_valid,"ordinary mapping extends independently of the prior-update flag");
    }
    extension(root,false);
    extension(root,true);
    {
      Fixture f(root,"numerical",true,true);
      f.scan({0},0.);
      auto &manager=*f.mapper->voxelmap_manager;
      StatesGroup singular;
      singular.cov.setZero(); manager.state_=singular;
      check(!manager.StateEstimation(singular) && manager.numerical_failure_ && manager.effct_feat_num_>=6,
            "non-finite solution is distinguished from missing constraints");
      f.scan({0},0.);
      check(f.mapper->tracking_valid && !manager.numerical_failure_,"a later valid estimate clears the numerical-failure reason");
    }
    {
      Fixture f(root,"failed_save",true,true);
      f.scan({0,1},1.5);
      const auto error=f.failThroughRun(true);
      check(error.find("lost constraints")!=std::string::npos,"a final-save error does not replace the original tracking failure");
    }
    std::filesystem::remove_all(root);
    rclcpp::shutdown();
    std::cout<<"PASS "<<checks<<" prior extension checks"<<std::endl;
    return 0;
  } catch (const std::exception &error) {
    rclcpp::shutdown();
    std::cerr<<"FAIL: "<<error.what()<<"; fixture: "<<root<<std::endl;
    return 1;
  }
}
