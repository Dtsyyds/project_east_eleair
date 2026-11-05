/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <ctime>
#include <exception>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <sys/timex.h>
#include <vikit/camera_loader.h>
#include <tf2/exceptions.h>

using namespace Sophus;

namespace
{
bool hasFiniteXYZ(const PointCloudXYZI &cloud)
{
  return std::all_of(cloud.begin(), cloud.end(), [](const PointType &point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
  });
}

void validateSystemTime()
{
  // modes=0 only queries the kernel clock state; it never adjusts the clock.
  struct timex clock_status {};
  const int clock_state = ::adjtimex(&clock_status);
  if (clock_state < 0)
  {
    throw std::system_error(errno, std::generic_category(), "时间校验失败：无法读取系统时钟同步状态");
  }

  // Leap-second states are synchronized states too. Unknown states fail closed.
  if (clock_state > TIME_WAIT || (clock_status.status & (STA_UNSYNC | STA_CLOCKERR)) != 0)
  {
    std::ostringstream reason;
    reason << "时间校验失败：系统未报告时间同步成功或存在时钟异常（state="
           << clock_state << ", status=0x" << std::hex << clock_status.status
           << "）。请完成系统校时后重新启动建图。";
    throw std::runtime_error(reason.str());
  }
}
}  // namespace

LIVMapper::LIVMapper(rclcpp::Node::SharedPtr &node, std::string node_name, const rclcpp::NodeOptions & options)
    : node(std::make_shared<rclcpp::Node>(node_name, options)),
      extT(0, 0, 0),
      extR(M3D::Identity())
{
  //---------参数初始化---------//
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);
  //雷达、摄像头外参

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());
  // 重置

  readParameters(this->node);
  if (gravity_align_en && !imu_en)
  {
    throw std::invalid_argument("uav.gravity_align_en=true requires imu.imu_en=true.");
  }
  initializeMapSession();
  VoxelMapConfig voxel_config;
  loadVoxelConfig(this->node, voxel_config);
  if (locate_in_prior_map && !prior_map_update_en && voxel_config.map_sliding_en)
  {
    RCLCPP_WARN(this->node->get_logger(), "Disabling map sliding in prior-map localization to preserve the fixed prior.");
    voxel_config.map_sliding_en = false;
  }
  RCLCPP_INFO(this->node->get_logger(), "Mapping mode: %s; map sliding: %s.",
      !locate_in_prior_map ? "mapping" : prior_map_update_en ? "prior_extension" : "prior_localization",
      voxel_config.map_sliding_en ? "enabled" : "disabled");

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  feats_down_prior_map.reset(new PointCloudXYZI());//『GT』
  prior_map.reset(new pcl::PointCloud<PointType>());//『GT』

  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents(this->node);          // initialize components errors
  path.header.stamp = this->node->now();
  path.header.frame_id = "camera_init"; // world frame 可根据需要更改
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->node);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

LIVMapper::~LIVMapper() = default;

bool LIVMapper::isPcdSavingEnabled() const
{
  return pcd_save_en && (!locate_in_prior_map || pcd_save_in_localization);
}

void LIVMapper::readParameters(rclcpp::Node::SharedPtr &node)
{
  // declare parameters
  auto try_declare = [node]<typename ParameterT>(const std::string & name,
    const ParameterT & default_value)
  {
    if (!node->has_parameter(name))
    {
      return node->declare_parameter<ParameterT>(name, default_value);
    }
    else
    {
      return node->get_parameter(name).get_value<ParameterT>();
    }
  };

  const auto default_workspace_root = (std::filesystem::path(ROOT_DIR) / "../..").lexically_normal();
  const std::filesystem::path workspace_root(try_declare.template operator()<std::string>(
      "workspace_root", default_workspace_root.string()));
  if (workspace_root.empty() || !workspace_root.is_absolute())
    throw std::invalid_argument("workspace_root must be a non-empty absolute directory path.");
  const auto resolve_directory = [&workspace_root](const std::string &value) {
    const std::filesystem::path directory(value);
    if (directory.empty()) return value;  // Preserve validation of empty enabled paths below.
    return (directory.is_absolute() ? directory : workspace_root / directory).lexically_normal().string();
  };

  // declare parameter
  try_declare.template operator()<std::string>("common.lid_topic", "/livox/lidar");
  try_declare.template operator()<std::string>("common.imu_topic", "/livox/imu");
  try_declare.template operator()<bool>("common.ros_driver_bug_fix", false);
  try_declare.template operator()<int>("common.img_en", 1);
  try_declare.template operator()<int>("common.lidar_en", 1);
  try_declare.template operator()<std::string>("common.img_topic", "/left_camera/image");

  try_declare.template operator()<bool>("vio.normal_en", true);
  try_declare.template operator()<bool>("vio.inverse_composition_en", false);
  try_declare.template operator()<int>("vio.max_iterations", 5);
  // Accept both historical integer YAML values and floating-point values.
  if (!node->has_parameter("vio.img_point_cov")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    node->declare_parameter("vio.img_point_cov", rclcpp::ParameterValue(100.0), descriptor);
  }
  try_declare.template operator()<bool>("vio.raycast_en", false);
  try_declare.template operator()<bool>("vio.exposure_estimate_en", true);
  try_declare.template operator()<double>("vio.inv_expo_cov", 0.1);
  try_declare.template operator()<int>("vio.grid_size", 5);
  try_declare.template operator()<int>("vio.grid_n_height", 17);
  try_declare.template operator()<int>("vio.patch_pyrimid_level", 4);
  try_declare.template operator()<int>("vio.patch_size", 8);
  // Accept both historical integer YAML values and floating-point values.
  if (!node->has_parameter("vio.outlier_threshold")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    node->declare_parameter("vio.outlier_threshold", rclcpp::ParameterValue(100.0), descriptor);
  }
  try_declare.template operator()<double>("time_offset.exposure_time_init", 0.0);
  try_declare.template operator()<double>("time_offset.img_time_offset", 0.0);
  try_declare.template operator()<bool>("uav.imu_rate_odom", false);
  try_declare.template operator()<bool>("uav.gravity_align_en", false);

  try_declare.template operator()<std::string>("evo.seq_name", "01");
  try_declare.template operator()<bool>("evo.pose_output_en", false);
  try_declare.template operator()<double>("imu.gyr_cov", 1.0);
  try_declare.template operator()<double>("imu.acc_cov", 1.0);
  try_declare.template operator()<int>("imu.imu_int_frame", 30);
  try_declare.template operator()<bool>("imu.imu_en", true);
  try_declare.template operator()<bool>("imu.gravity_est_en", true);
  try_declare.template operator()<bool>("imu.ba_bg_est_en", true);

  try_declare.template operator()<double>("preprocess.blind", 0.01);
  try_declare.template operator()<double>("preprocess.filter_size_surf", 0.5);
  try_declare.template operator()<double>("preprocess.filter_size_map", 0.1);//『GT』
  try_declare.template operator()<int>("preprocess.lidar_type", AVIA);
  try_declare.template operator()<int>("preprocess.scan_line",6);
  try_declare.template operator()<int>("preprocess.point_filter_num", 3);
  try_declare.template operator()<int>("preprocess.scan_rate", 10);
  try_declare.template operator()<bool>("preprocess.feature_extract_enabled", false);

  try_declare.template operator()<bool>("time_check.enabled", true);
  try_declare.template operator()<int>("pcd_save.interval", -1);
  try_declare.template operator()<bool>("pcd_save.pcd_save_en", false);
  try_declare.template operator()<bool>("pcd_save.pcd_save_en_relocate", false);
  try_declare.template operator()<std::string>("pcd_save.output_dir", "PCD/priori");
  try_declare.template operator()<std::string>("pcd_save.output_dir_relocate", "PCD/relocate");
  try_declare.template operator()<bool>("pcd_save.colmap_output_en", false);
  try_declare.template operator()<double>("pcd_save.filter_size_pcd", 0.5);
  try_declare.template operator()<vector<double>>("extrin_calib.extrinsic_T", vector<double>{});
  try_declare.template operator()<vector<double>>("extrin_calib.extrinsic_R", vector<double>{});
  try_declare.template operator()<vector<double>>("extrin_calib.Pcl", vector<double>{});
  try_declare.template operator()<vector<double>>("extrin_calib.Rcl", vector<double>{});
  try_declare.template operator()<double>("debug.plot_time", -10);
  try_declare.template operator()<int>("debug.frame_cnt", 6);

  try_declare.template operator()<double>("publish.blind_rgb_points", 0.01);
  try_declare.template operator()<int>("publish.pub_scan_num", 1);
  try_declare.template operator()<bool>("publish.pub_effect_point_en", false);
  try_declare.template operator()<bool>("publish.dense_map_en", false);
  try_declare.template operator()<std::string>("publish.map_frame_id", "map");

  try_declare.template operator()<bool>("locate_in_prior_map", false);//『GT』
  try_declare.template operator()<bool>("prior_map_update_en", false);
  try_declare.template operator()<string>("prior_map_path", "PCD/map");
  try_declare.template operator()<string>("prior_map_name", "");

  try_declare.template operator()<double>("time_offset.imu_time_offset", 0.0);
  try_declare.template operator()<double>("imu.b_acc_cov", 0.0001);
  try_declare.template operator()<double>("imu.b_gyr_cov", 0.0001);
  try_declare.template operator()<bool>("log.sensor", false);
  try_declare.template operator()<bool>("log.lio", false);
  try_declare.template operator()<bool>("log.timing", false);
  try_declare.template operator()<int>("localization.max_unconstrained_frames", 5);
  try_declare.template operator()<int>("pcd_save.type", -1);
  try_declare.template operator()<bool>("image_save.img_save_en", false);
  try_declare.template operator()<int>("image_save.interval", 1);
  try_declare.template operator()<std::string>("image_save.output_dir", "images");

  // get parameter
  this->node->get_parameter("common.lid_topic", lid_topic);
  this->node->get_parameter("common.imu_topic", imu_topic);
  this->node->get_parameter("common.ros_driver_bug_fix", ros_driver_fix_en);
  this->node->get_parameter("common.img_en", img_en);
  this->node->get_parameter("common.lidar_en", lidar_en);
  this->node->get_parameter("common.img_topic", img_topic);

  this->node->get_parameter("vio.normal_en", normal_en);
  this->node->get_parameter("vio.inverse_composition_en", inverse_composition_en);
  this->node->get_parameter("vio.max_iterations", max_iterations);
  {
    const auto value = node->get_parameter("vio.img_point_cov");
    IMG_POINT_COV = value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
        static_cast<double>(value.as_int()) : value.as_double();
    if (!std::isfinite(IMG_POINT_COV) || IMG_POINT_COV <= 0.0) throw std::invalid_argument("vio.img_point_cov must be finite and positive.");
  }
  this->node->get_parameter("vio.raycast_en", raycast_en);
  this->node->get_parameter("vio.exposure_estimate_en", exposure_estimate_en);
  this->node->get_parameter("vio.inv_expo_cov", inv_expo_cov);
  this->node->get_parameter("vio.grid_size", grid_size);
  this->node->get_parameter("vio.grid_n_height", grid_n_height);
  this->node->get_parameter("vio.patch_pyrimid_level", patch_pyrimid_level);
  this->node->get_parameter("vio.patch_size", patch_size);
  {
    const auto value = node->get_parameter("vio.outlier_threshold");
    outlier_threshold = value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ?
        static_cast<double>(value.as_int()) : value.as_double();
    if (!std::isfinite(outlier_threshold) || outlier_threshold <= 0.0) throw std::invalid_argument("vio.outlier_threshold must be finite and positive.");
  }
  this->node->get_parameter("time_offset.exposure_time_init", exposure_time_init);
  this->node->get_parameter("time_offset.img_time_offset", img_time_offset);
  this->node->get_parameter("uav.imu_rate_odom", imu_prop_enable);
  this->node->get_parameter("uav.gravity_align_en", gravity_align_en);

  this->node->get_parameter("evo.seq_name", seq_name);
  this->node->get_parameter("evo.pose_output_en", pose_output_en);
  this->node->get_parameter("imu.gyr_cov", gyr_cov);
  this->node->get_parameter("imu.acc_cov", acc_cov);
  this->node->get_parameter("imu.imu_int_frame", imu_int_frame);
  this->node->get_parameter("imu.imu_en", imu_en);
  this->node->get_parameter("imu.gravity_est_en", gravity_est_en);
  this->node->get_parameter("imu.ba_bg_est_en", ba_bg_est_en);

  this->node->get_parameter("preprocess.blind", p_pre->blind);
  this->node->get_parameter("preprocess.filter_size_surf", filter_size_surf_min);
  this->node->get_parameter("preprocess.filter_size_map", filter_size_map_min);//『GT』
  this->node->get_parameter("preprocess.lidar_type", p_pre->lidar_type);
  this->node->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
  this->node->get_parameter("preprocess.scan_rate", p_pre->SCAN_RATE);
  this->node->get_parameter("preprocess.point_filter_num", p_pre->point_filter_num);
  this->node->get_parameter("preprocess.feature_extract_enabled", p_pre->feature_enabled);

  this->node->get_parameter("time_check.enabled", time_check_enabled);
  this->node->get_parameter("pcd_save.interval", pcd_save_interval);
  this->node->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
  this->node->get_parameter("pcd_save.pcd_save_en_relocate", pcd_save_in_localization);
  this->node->get_parameter("pcd_save.output_dir", pcd_output_dir);
  this->node->get_parameter("pcd_save.colmap_output_en", colmap_output_en);
  this->node->get_parameter("pcd_save.filter_size_pcd", filter_size_pcd);
  this->node->get_parameter("extrin_calib.extrinsic_T", extrinT);
  this->node->get_parameter("extrin_calib.extrinsic_R", extrinR);
  this->node->get_parameter("extrin_calib.Pcl", cameraextrinT);
  this->node->get_parameter("extrin_calib.Rcl", cameraextrinR);
  this->node->get_parameter("debug.plot_time", plot_time);
  this->node->get_parameter("debug.frame_cnt", frame_cnt);

  this->node->get_parameter("publish.blind_rgb_points", blind_rgb_points);
  this->node->get_parameter("publish.pub_scan_num", pub_scan_num);
  this->node->get_parameter("publish.pub_effect_point_en", pub_effect_point_en);
  this->node->get_parameter("publish.dense_map_en", dense_map_en);
  this->node->get_parameter("publish.map_frame_id", map_frame_id_);

  this->node->get_parameter("locate_in_prior_map", locate_in_prior_map);//『GT』
  this->node->get_parameter("prior_map_update_en", prior_map_update_en);
  this->node->get_parameter("prior_map_path", prior_map_path); //『GT』
  prior_map_path = resolve_directory(prior_map_path);
  node->get_parameter("time_offset.imu_time_offset", imu_time_offset);
  node->get_parameter("imu.b_acc_cov", b_acc_cov);
  node->get_parameter("imu.b_gyr_cov", b_gyr_cov);
  node->get_parameter("log.sensor", log_sensor);
  node->get_parameter("log.lio", log_lio);
  node->get_parameter("log.timing", log_timing);
  node->get_parameter("localization.max_unconstrained_frames", max_unconstrained_frames);
  node->get_parameter("pcd_save.type", pcd_save_type);
  // Resolve automatic coordinates and mode-specific paths once at startup.
  if (pcd_save_type == -1) pcd_save_type = locate_in_prior_map ? 2 : 0;
  if (locate_in_prior_map)
  {
    node->get_parameter("pcd_save.output_dir_relocate", pcd_output_dir);
    const std::filesystem::path directory(prior_map_path);
    const std::filesystem::path filename(node->get_parameter("prior_map_name").as_string());
    if (!directory.is_absolute() || filename.empty() || filename.has_parent_path() ||
        filename == "." || filename == "..")
      throw std::invalid_argument("Prior localization requires an absolute prior_map_path directory and a plain prior_map_name filename.");
    prior_map_path = (directory / filename).lexically_normal().string();
  }
  pcd_output_dir = resolve_directory(pcd_output_dir);
  node->get_parameter("image_save.img_save_en", img_save_en);
  node->get_parameter("image_save.interval", img_save_interval);
  node->get_parameter("image_save.output_dir", image_output_dir);
  image_output_dir = resolve_directory(image_output_dir);
  p_pre->log_points = log_sensor;
  if (extrinR.size() != 9 || extrinT.size() != 3) throw std::invalid_argument("LiDAR-to-IMU extrinsics require 9 rotation and 3 translation values.");
  M3D rotation; rotation << MAT_FROM_ARRAY(extrinR);
  if (!rotation.allFinite() || (rotation.transpose()*rotation-M3D::Identity()).norm() > 1e-6 ||
      std::abs(rotation.determinant()-1.0) > 1e-6 ||
      !std::all_of(extrinT.begin(), extrinT.end(), [](double v){ return std::isfinite(v); }))
    throw std::invalid_argument("Invalid LiDAR-to-IMU rigid transform.");
  if (!std::isfinite(imu_time_offset) || !std::isfinite(b_acc_cov) || !std::isfinite(b_gyr_cov) ||
      b_acc_cov < 0 || b_gyr_cov < 0 || !std::isfinite(acc_cov) || !std::isfinite(gyr_cov) ||
      acc_cov < 0 || gyr_cov < 0 || !std::isfinite(filter_size_surf_min) || filter_size_surf_min <= 0 ||
      p_pre->point_filter_num < 1 || p_pre->N_SCANS < 1 || p_pre->N_SCANS > 128 ||
      pcd_save_interval == 0 || max_unconstrained_frames < 1 || img_save_interval < 1 ||
      pcd_save_type < 0 || pcd_save_type > 2 || pub_scan_num < 1)
    throw std::invalid_argument("Invalid noise, filter, save interval/type or localization limit parameter.");
  if (img_save_en && !img_en) throw std::invalid_argument("image_save requires common.img_en=1.");
  if (colmap_output_en && (!img_en || pcd_save_type != 0 || !isPcdSavingEnabled()))
    throw std::invalid_argument("COLMAP requires vision and enabled local-world PCD saving (pcd_save.type=0).");
  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

void LIVMapper::initializeComponents(rclcpp::Node::SharedPtr &node) 
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min); //设置体素滤波器的叶子大小，LIVMapper.h中设置为0,应该是可以自己设置的

  // extrinT.assign({0.04165, 0.02326, -0.0284});
  // extrinR.assign({1, 0, 0, 0, 1, 0, 0, 0, 1});
  // cameraextrinT.assign({0.0194384, 0.104689,-0.0251952});
  // cameraextrinR.assign({0.00610193,-0.999863,-0.0154172,-0.00615449,0.0153796,-0.999863,0.999962,0.00619598,-0.0060598});

  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  if (img_en)
  {
  if (!vk::camera_loader::loadFromRosNs(this->node, "camera", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->initializeVIO();
  }

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D::Constant(b_gyr_cov));
  p_imu->set_acc_bias_cov(V3D::Constant(b_acc_cov));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO; //三种模式自动根据config中的配置有无选择
}

void LIVMapper::initializeMapSession()
{
  if (time_check_enabled)
  {
    validateSystemTime();
  }
  else
  {
    RCLCPP_WARN(this->node->get_logger(), "时间校验已关闭：未验证系统时间准确性，将使用当前系统时间命名地图。");
  }

  // Use the local system wall clock once per session, independently of ROS /clock.
  const std::time_t session_time = std::time(nullptr);
  std::tm local_time {};
  if (session_time == std::time_t(-1) || ::localtime_r(&session_time, &local_time) == nullptr)
  {
    throw std::runtime_error("无法获取建图启动时的本地日期与时间，不能生成地图文件名前缀。");
  }

  std::ostringstream display_time;
  display_time << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S %Z (%z)");
  if (time_check_enabled)
  {
    RCLCPP_INFO(this->node->get_logger(), "时间校验通过：系统报告已同步，当前本地时间 %s", display_time.str().c_str());
  }
  else
  {
    RCLCPP_INFO(this->node->get_logger(), "当前本地时间（未经校验）：%s", display_time.str().c_str());
  }

  std::ostringstream prefix;
  prefix << "map_" << std::put_time(&local_time, "%m%d_%H%M");
  pcd_file_prefix = prefix.str();
  RCLCPP_INFO(this->node->get_logger(), "本次地图文件前缀：%s", pcd_file_prefix.c_str());
}

void LIVMapper::initializeFiles() 
{
  if (isPcdSavingEnabled())
  {
    const std::filesystem::path output_dir(pcd_output_dir);
    if (!output_dir.is_absolute())
    {
      throw std::invalid_argument("pcd_save.output_dir must be a non-empty absolute directory path.");
    }
    std::filesystem::create_directories(output_dir);
    RCLCPP_INFO(this->node->get_logger(), "PCD output directory: %s", pcd_output_dir.c_str());
  }
  else if (!pcd_save_en)
  {
    RCLCPP_INFO(this->node->get_logger(), "PCD 保存已关闭：pcd_save.pcd_save_en=false。");
  }
  else
  {
    RCLCPP_INFO(this->node->get_logger(), "定位模式 PCD 保存已禁用：pcd_save.pcd_save_en_relocate=false，不累积或写出观测点云。");
  }

  if (pose_output_en) std::filesystem::create_directories(std::string(ROOT_DIR) + "Log/result");
  if (img_save_en) {
    if (!std::filesystem::path(image_output_dir).is_absolute()) throw std::invalid_argument("image_save.output_dir must be absolute.");
    std::filesystem::create_directories(image_output_dir);
  }
  if (colmap_output_en) {
    const auto output = std::filesystem::path(pcd_output_dir) / (pcd_file_prefix + "_colmap");
    std::filesystem::create_directories(output / "images");
    std::filesystem::create_directories(output / "sparse/0");
    vio_manager->colmap_output_dir = output.string();
    fout_points.open((output / "sparse/0/points3D.txt").string(), std::ios::out);
    if (!fout_points) throw std::runtime_error("Cannot create COLMAP points file.");
  }
  if(isPcdSavingEnabled() && pcd_save_interval > 0)
  {
    fout_pcd_pos.open((std::filesystem::path(pcd_output_dir) / (pcd_file_prefix + "_scans_pos.txt")).string(), std::ios::out);
    if (!fout_pcd_pos) throw std::runtime_error("Cannot create segment pose file.");
  }
  if (log_lio) {
    std::filesystem::create_directories(std::string(ROOT_DIR) + "Log");
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
  }
}

void LIVMapper::initializeSubscribersAndPublishers(rclcpp::Node::SharedPtr &node)
{
  image_transport::ImageTransport it(this->node);
  if (!locate_in_prior_map) initializeSensorSubscriptions();
  /*『GT』*/
  if (locate_in_prior_map) 
  {
      initial_pose_service_ = this->node->create_service<fast_livo::srv::SetInitialPose>(
          "/fast_livo2/set_initial_pose",
          std::bind(&LIVMapper::set_initial_pose_cbk, this, std::placeholders::_1, std::placeholders::_2));
      initial_map_pose_pub_ = this->node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/fast_livo2/initial_map_pose", rclcpp::QoS(1).reliable().transient_local());
  }
  /*『GT』*/
  pubLaserCloudFullRes = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100);
  pubNormal = this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker", 100);
  pubSubVisualMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_visual_sub_map_before", 100);
  pubLaserCloudEffect = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100);
  pubOdomAftMapped = this->node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
  pubOdomAftMappedInMap = this->node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_in_map", 10);
  pubPath = this->node->create_publisher<nav_msgs::msg::Path>("/path", 10);
  pubState6 = this->node->create_publisher<std_msgs::msg::Float32MultiArray>("/fast_livo2/state6", 50);
  pubState6ImuProp = this->node->create_publisher<std_msgs::msg::Float32MultiArray>("/fast_livo2/state6_imu_prop", 200);
  plane_pub = this->node->create_publisher<visualization_msgs::msg::Marker>("/planner_normal", 1);
  voxel_pub = this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj", 100);
  pubLaserCloudDynRmed = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_dbg_hist", 100);
  mavros_pose_publisher = this->node->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
  if (img_en) pubImage = it.advertise("/rgb_img", 1);
  pubImuPropOdom = this->node->create_publisher<nav_msgs::msg::Odometry>("/LIVO2/imu_propagate", 10000);
  if (imu_prop_enable) imu_prop_timer = this->node->create_wall_timer(0.004s, std::bind(&LIVMapper::imu_prop_callback, this));
  voxelmap_manager->voxel_map_pub_= this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/planes", rclcpp::QoS(1).reliable().transient_local());
 
  tracking_valid_pub_ = this->node->create_publisher<std_msgs::msg::Bool>("/fast_livo2/tracking_valid", rclcpp::QoS(1).transient_local());
  tracking_valid_pub_->publish(std_msgs::msg::Bool());
  state6_stamped_pub_ = this->node->create_publisher<fast_livo::msg::State6Stamped>("/fast_livo2/state6_stamped", 10);
  state6_imu_stamped_pub_ = this->node->create_publisher<fast_livo::msg::State6Stamped>("/fast_livo2/state6_imu_prop_stamped", 10);
  initializePriorMap();
}

void LIVMapper::initializeSensorSubscriptions()
{
  if (p_pre->lidar_type == AVIA) {
    sub_pcl = this->node->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, rclcpp::SensorDataQoS().keep_last(10), std::bind(&LIVMapper::livox_pcl_cbk, this, std::placeholders::_1));
  } else {
    sub_pcl = this->node->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS().keep_last(10), std::bind(&LIVMapper::standard_pcl_cbk, this, std::placeholders::_1));
  }
  if (imu_en) sub_imu = this->node->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS().keep_last(200), std::bind(&LIVMapper::imu_cbk, this, std::placeholders::_1));
  if (img_en) sub_img = this->node->create_subscription<sensor_msgs::msg::Image>(
      img_topic, rclcpp::SensorDataQoS().keep_last(5), std::bind(&LIVMapper::img_cbk, this, std::placeholders::_1));
}

void LIVMapper::initializePriorMap()
{
  if (!locate_in_prior_map) return;

  if (prior_map_path.empty() || !std::filesystem::path(prior_map_path).is_absolute())
  {
    throw std::invalid_argument("locate_in_prior_map=true requires a non-empty prior_map_path.");
  }
  const float leaf_size = static_cast<float>(filter_size_map_min);
  if (!std::isfinite(leaf_size) || leaf_size <= 0.0f)
  {
    throw std::invalid_argument("preprocess.filter_size_map must be finite and positive when loading a prior map.");
  }

  RCLCPP_INFO(this->node->get_logger(), "Loading prior map: %s", prior_map_path.c_str());
  if (pcl::io::loadPCDFile<PointType>(prior_map_path, *prior_map) != 0)
  {
    throw std::runtime_error("Failed to load prior PCD map: " + prior_map_path);
  }
  if (prior_map->empty())
  {
    throw std::runtime_error("Prior PCD map contains no points: " + prior_map_path);
  }
  if (!hasFiniteXYZ(*prior_map))
  {
    throw std::runtime_error("Prior PCD map contains non-finite XYZ values: " + prior_map_path);
  }

  downSizeFilterMap.setLeafSize(leaf_size, leaf_size, leaf_size);
  downSizeFilterMap.setInputCloud(prior_map);
  feats_down_prior_map->clear();
  downSizeFilterMap.filter(*feats_down_prior_map);
  if (feats_down_prior_map->empty() || !hasFiniteXYZ(*feats_down_prior_map))
  {
    throw std::runtime_error("Prior map is empty or has non-finite XYZ after downsampling: " + prior_map_path);
  }
  RCLCPP_INFO(this->node->get_logger(), "Prior map ready: %zu raw points, %zu downsampled points.",
              prior_map->size(), feats_down_prior_map->size());
}

void LIVMapper::initializeVoxelMap()
{
  const auto &body = voxelmap_manager->feats_down_body_;
  const auto &world = voxelmap_manager->feats_down_world_;
  if (!body || !world || world->empty() || body->size() != world->size())
  {
    throw std::runtime_error("Cannot initialize voxel map: point clouds must be non-empty and have matching sizes.");
  }
  if (!hasFiniteXYZ(*body) || !hasFiniteXYZ(*world))
  {
    throw std::runtime_error("Cannot initialize voxel map: point cloud contains non-finite XYZ values.");
  }
  const auto &config = voxelmap_manager->config_setting_;
  if (!std::isfinite(config.max_voxel_size_) || config.max_voxel_size_ <= 0.0)
  {
    throw std::invalid_argument("lio.voxel_size must be finite and positive when initializing a voxel map.");
  }
  if (config.max_layer_ < 0 || static_cast<size_t>(config.max_layer_) >= config.layer_init_num_.size())
  {
    throw std::invalid_argument("lio.layer_init_num must contain at least lio.max_layer + 1 entries; lio.max_layer must be non-negative.");
  }

  voxelmap_manager->BuildVoxelMap();
  if (voxelmap_manager->voxel_map_.empty())
  {
    throw std::runtime_error("Voxel map initialization produced no voxels.");
  }
  const auto plane_count = voxelmap_manager->countValidPlanes();
  if (locate_in_prior_map && plane_count == 0)
    throw std::runtime_error("Prior voxel map has zero valid planes. Reduce preprocess.filter_size_map (recommended 0.1 m) or use a map with usable planar geometry.");
  RCLCPP_INFO(this->node->get_logger(), "Voxel initialization: %zu voxels, %zu valid planes.", voxelmap_manager->voxel_map_.size(), plane_count);
  lidar_map_inited = true;
}

bool LIVMapper::prepareInitialMapPose()
{
  if (initial_alignment_ready) return true;
  if (!initial_pose_received) return false;
  if (gravity_align_en && (p_imu->imu_need_init || !gravity_align_finished)) return false;

  const auto &position = initial_pose.pose.pose.position;
  const auto &orientation = initial_pose.pose.pose.orientation;
  Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
  if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-12)
  {
    throw std::runtime_error("Cannot prepare initial map pose: invalid initial IMU quaternion.");
  }
  q.normalize();
  Eigen::Matrix4d T_map_initial_imu = Eigen::Matrix4d::Identity();
  T_map_initial_imu.block<3, 3>(0, 0) = q.toRotationMatrix();
  T_map_initial_imu.block<3, 1>(0, 3) = V3D(position.x, position.y, position.z);
  Eigen::Matrix4d T_camera_init_initial_imu = Eigen::Matrix4d::Identity();
  if (gravity_align_en) T_camera_init_initial_imu.block<3, 3>(0, 0) = gravity_alignment_rotation_;

  // One fixed transform drives both prior-map coordinates and the final TF pose.
  initial_map_transform_ = T_map_initial_imu * T_camera_init_initial_imu.inverse();
  if (!initial_map_transform_.allFinite())
  {
    throw std::runtime_error("Cannot prepare initial map pose: map-to-camera_init transform is not finite.");
  }
  initial_alignment_ready = true;
  RCLCPP_INFO(this->node->get_logger(), "Final map -> camera_init alignment computed.");
  return true;
}

void LIVMapper::publishInitialMapPose()
{
  if (!initial_alignment_ready || !lidar_map_inited || !initial_map_pose_pub_)
  {
    throw std::logic_error("Initial map pose can only be published after successful prior-map initialization.");
  }
  auto pose = initial_pose;
  pose.header.frame_id = map_frame_id_;
  pose.header.stamp = this->node->now();
  pose.pose.pose.position.x = initial_map_transform_(0, 3);
  pose.pose.pose.position.y = initial_map_transform_(1, 3);
  pose.pose.pose.position.z = initial_map_transform_(2, 3);
  Eigen::Quaterniond q(initial_map_transform_.block<3, 3>(0, 0));
  q.normalize();
  pose.pose.pose.orientation.x = q.x();
  pose.pose.pose.orientation.y = q.y();
  pose.pose.pose.orientation.z = q.z();
  pose.pose.pose.orientation.w = q.w();
  initial_map_pose_pub_->publish(pose);
  writeMapMetadata();
  RCLCPP_INFO(this->node->get_logger(),
              "Published /fast_livo2/initial_map_pose after prior-map initialization; the device may now move.");
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

//ez(0,0,-1) 表示目标重力方向（在常见右手坐标系里重力向下），gz(_state.gravity) 是当前估计的重力向量
void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    if (!gz.allFinite() || !std::isfinite(gz.norm()) || gz.norm() < 1e-6)
    {
      throw std::runtime_error("Cannot align gravity: initialized gravity vector is zero or non-finite.");
    }
    Eigen::Quaterniond G_q_I0 = Eigen::Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();
    if (!G_R_I0.allFinite()) throw std::runtime_error("Gravity alignment produced a non-finite rotation.");
    gravity_alignment_rotation_ = G_R_I0;

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;//把当前状态赋值给 state_propagat，作为后续建图处理的先验状态。
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}


void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}

//VIO先忽略
void LIVMapper::handleVIO() 
{
  if (!tracking_valid) return;
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
    
  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr)) 
  {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }
    
  std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  publish_img_rgb(pubImage, vio_manager);

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;

  publish_state6(pubState6, _state.rot_end, p_imu->unbiased_gyr);
}

void LIVMapper::handleLIO() 
{    
  //总体流程：检查输入点云 → 下采样 → 变换到世界系 → 初始化/更新地图 → 状态估计 → 更新协方差/体素 → 发布与保存结果。
  
  //=======================日志记录============================//
  euler_cur = RotMtoEuler(_state.rot_end); //把当前旋转转换为欧拉角（弧度），用于日志或发布。
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
  //fout_pre << ...; 把时间、欧拉角（转为度乘以57.3）、位置、速度、陀螺/加速度偏置、逆曝光时间写入预览日志文件（fout_pre）。         
  
  if (!feats_undistort || feats_undistort->empty()) //检查去畸变特征点云是否为空或未初始化。
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }


  //=======================点云下采样与计时============================//
  double t0 = omp_get_wtime();//记录当前时间，用于后续计算处理时间。

  downSizeFilterSurf.setInputCloud(feats_undistort); //设置输入点云为去畸变特征点云。
  feats_down_body->clear();                          //清空下采样后的点云容器
  downSizeFilterSurf.filter(*feats_down_body);//执行体素网格滤波器进行下采样，结果存储在feats_down_body中。
  
  double t_down = omp_get_wtime();//记录下采样完成后的时间。


  //=======================变换到世界系与地图初始化============================//
  feats_down_size = feats_down_body->points.size();//记录下采样后点云的大小（数量）。
  voxelmap_manager->feats_down_body_ = feats_down_body;//将下采样后的点云存储在体素地图管理器中，供后续处理使用。
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);//把下采样点云从机体系变换到世界系
                                                                                    //（使用当前 _state 的旋转和平移以及外参 extR/extT）。
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  //feats_down_world 是变换到世界系的下采样点云，供地图更新和状态估计使用。
  //feats_down_size 是下采样点云的数量。
  //feats_down_body 是下采样后的机体系点云，供后续处理使用。
  //机体系是指相对于传感器本身的坐标系，世界系是指全局参考坐标系。


  //=======================首次构建体素地图============================//
  if (!lidar_map_inited) 
  {
    /* 『GT』 */
    if (locate_in_prior_map)
    {
    // 1) 等初始 IMU 位姿与重力旋转就绪，使用最终 map -> camera_init 变换的逆。
      if (!feats_down_prior_map || feats_down_prior_map->empty())
      {
        throw std::runtime_error("Cannot initialize prior voxel map: downsampled prior map is empty.");
      }

    if (!prepareInitialMapPose())
    {
      RCLCPP_INFO_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 3000,
                           "Waiting for initial IMU pose and gravity alignment before prior-map initialization.");
      return;
    }

    Eigen::Matrix4f T_init_inv_f = initial_map_transform_.inverse().cast<float>();

    PointCloudXYZI::Ptr tmp_prior(new PointCloudXYZI);
    pcl::transformPointCloud(*feats_down_prior_map, *tmp_prior, T_init_inv_f);

    // 2) 可视化 / 发布变换后的先验地图（与 FAST-LIO2 完全等价）
    sensor_msgs::msg::PointCloud2 prior_map_msg;
    pcl::toROSMsg(*tmp_prior, prior_map_msg);
    prior_map_msg.header.stamp = this->node->get_clock()->now(); // 或 this->get_clock()->now()，按你的类成员使用
    prior_map_msg.header.frame_id = "camera_init";
    pubLaserCloudMap->publish(prior_map_msg);

    // 3) 将变换后的 prior map 交给 voxelmap_manager 做 BuildVoxelMap 初始化
    voxelmap_manager->feats_down_world_ = tmp_prior; // BuildVoxelMap 会读取 feats_down_world_
    voxelmap_manager->feats_down_body_  = tmp_prior; // 以简洁处理（FAST-LIO2 用同一指针）
    voxelmap_manager->feats_down_size_  = static_cast<int>(tmp_prior->points.size());

    initializeVoxelMap();

    RCLCPP_INFO(this->node->get_logger(), "Initialized voxel map from prior map (locate_in_prior_map).");
    publishInitialMapPose();

    // 注意：初始化完毕后直接 return，防止后续重复 Build / 插入当前帧点云
    return;
  }
  // else if (feats_down_size > 5)
  // {
  //   // 常规首帧初始化（与原逻辑等价）：
  //   // 把当前帧变换到世界系的点云拷贝给 voxelmap_manager 并构建体素地图
  //   voxelmap_manager->feats_down_world_.reset(new pcl::PointCloud<pcl::PointXYZI>(*feats_down_world)); // 深拷贝（可选）
  //   voxelmap_manager->feats_down_body_.reset(new pcl::PointCloud<pcl::PointXYZI>(*feats_down_body));
  //   voxelmap_manager->feats_down_size_ = feats_down_size;
  //   voxelmap_manager->BuildVoxelMap();

  //   RCLCPP_INFO(this->node->get_logger(), "Initialized voxel map from first scan.");
  //   return;
  // }
  /* 『GT』 */

    initializeVoxelMap();//成功构建非空体素地图后，才设置初始化完成标志。
  }


  //=======================调用体素管理器做配准/状态估计============================//
  double t1 = omp_get_wtime();

  tracking_valid = voxelmap_manager->StateEstimation(state_propagat);
  std_msgs::msg::Bool validity;
  validity.data = tracking_valid;
  tracking_valid_pub_->publish(validity);
  if (!tracking_valid)
  {
    ekf_finish_once = false;
    pcl_w_wait_pub->clear();
    ++unconstrained_frames;
    const char *mode = !locate_in_prior_map ? "mapping" : prior_map_update_en ? "prior_extension" : "prior_localization";
    const char *reason = voxelmap_manager->numerical_failure_ ? "non-finite solution" : "insufficient valid constraints";
    RCLCPP_WARN_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 1000,
        "LIO state estimation failed (mode=%s, reason=%s, effective_points=%d, minimum=%d, consecutive_failures=%d/%d); suppressing pose/cloud/state output and map updates.",
        mode, reason, voxelmap_manager->effct_feat_num_, voxelmap_manager->config_setting_.min_effective_points_,
        unconstrained_frames, max_unconstrained_frames);
    if (locate_in_prior_map && unconstrained_frames >= max_unconstrained_frames)
    {
      std::ostringstream message;
      message << "Prior-map localization lost constraints (mode=" << mode << ", reason=" << reason
              << ", effective_points=" << voxelmap_manager->effct_feat_num_ << ", consecutive_failures="
              << unconstrained_frames << "/" << max_unconstrained_frames
              << "). Stop, verify the prior/initial pose, and restart mapping and relocalization.";
      throw std::runtime_error(message.str());
    }
    return;
  }
  unconstrained_frames = 0;
//把先验状态（state_propagat）传入 voxelmap_manager，
  //进行匹配/优化，更新体素地图与返回优化后的状态。

  _state = voxelmap_manager->state_;//更新当前状态为体素管理器优化后的状态。
  _pv_list = voxelmap_manager->pv_list_;//更新点云与协方差列表为体素管理器的结果。

  double t2 = omp_get_wtime();


  //=======================IMU 推断/触发标志============================//
  if (imu_prop_enable) //如果启用 IMU 推断
  {
    ekf_finish_once = true;//EKF是扩展卡尔曼滤波器（Extended Kalman Filter）的缩写。
                          //这里的 ekf_finish_once 标志表示 EKF 已经至少完成过一次更新。
    latest_ekf_state = _state;//将当前状态保存为最新的 EKF 状态。
    latest_ekf_time = LidarMeasures.last_lio_update_time;//将当前时间保存为最新的 EKF 时间。
    state_update_flg = true;//设置状态更新标志，通知 IMU 推断线程有新的状态可用。
  }//如果开启 IMU 推断，则将 EKF 完成标志与最新状态/时间记录下来，供另一个线程（imu_prop_callback）使用。
  //imu推断是指利用IMU数据在两个LIDAR帧之间进行状态估计的过程。


  //=======================姿态输出到文件（每帧 open — 性能问题）与体素地图更新============================//
  if (pose_output_en) 
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    } 
    else 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    }
    Eigen::Matrix4d outT;//定义一个4x4的矩阵，用于存储位姿变换矩阵。
    Eigen::Quaterniond q(_state.rot_end);//定义一个四元数，用于存储位姿的旋转部分。
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;//把时间戳、位置和四元数写入文件。
  }


  //=======================发布里程计与更新体素地图============================//
  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));//把当前旋转转换为欧拉角，再转换为四元数消息格式。


  //=======================再次计时并构建世界系点云（用于体素方差更新）============================//
  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());//创建一个新的点云指针，用于存储变换到世界系的点云。
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) //遍历变换到世界系的点云中的每个点
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;//更新体素管理器中对应点的世界系位置。
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];//获取该点的叉乘矩阵。
    M3D var = voxelmap_manager->body_cov_list_[i];//获取该点在机体系下的协方差矩阵。
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);//计算该点在世界系下的协方差矩阵。
    voxelmap_manager->pv_list_[i].var = var;
  }//更新每个点的协方差矩阵。以及体素地图的坐标转换


  //=======================体素地图更新与结果发布============================//
  // Insert the whole downsampled successful scan, including newly seen geometry.
  // Matching must finish before insertion; failed scans return above.
  if (!locate_in_prior_map || prior_map_update_en)
  {
    voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
    if (log_lio) std::cout << "[ LIO ] Update Voxel Map (" << (locate_in_prior_map ? "prior extension" : "mapping") << ")" << std::endl;
  }
  else
  {
    if (log_lio) std::cout << "[ LIO ] Locate in fixed prior map (no map update)" << std::endl;
  }

  _pv_list = voxelmap_manager->pv_list_;//更新当前点云与协方差列表为体素管理器的结果。
  
  double t4 = omp_get_wtime();


  //=======================滑动/裁剪地图（可选）============================//
  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();//如果启用地图滑动功能，调用 mapSliding 方法进行地图滑动处理。
  }//若启用滑动地图机制，则执行滑动窗口剔除远处体素或维持局部地图大小。


  publish_state6(pubState6, _state.rot_end, p_imu->unbiased_gyr);
  publish_odometry(pubOdomAftMapped);

  //==================准备并转换用于发布的完整点云============================//
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);//根据是否启用稠密地图选择使用去畸变特征点云还是下采样点云作为最终的全分辨率点云。
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);//把全分辨率点云从机体系转换到世界系。
  }
  *pcl_w_wait_pub = *laserCloudWorld;//将转换后的世界系点云存储在 pcl_w_wait_pub 中，供后续发布使用。


  //=======================结果发布与日志记录============================//
  if (!img_en) publish_frame_world(pubLaserCloudFullRes, vio_manager);//如果未启用图像处理，发布全分辨率点云到世界系。
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);//如果启用特征点发布，发布特征点到世界系。
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();//如果启用平面地图发布，调用体素地图管理器的 pubVoxelMap 方法发布平面地图。
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);//发布 MAVROS 位姿消息。MAVROS 是 ROS 与无人机通信的桥梁。

  frame_num++;//处理的帧数加一。
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  if (log_timing)
  {
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  }

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::writeMapMetadata()
{
  if (!isPcdSavingEnabled()) return;
  std::ofstream file(std::filesystem::path(pcd_output_dir)/(pcd_file_prefix + "_metadata.yaml"));
  if (!file) throw std::runtime_error("Cannot write PCD session metadata.");
  file << std::setprecision(17) << "coordinate_frame: " << (pcd_save_type==1 ? "per_scan_imu" : pcd_save_type==2 && locate_in_prior_map ? map_frame_id_ : "camera_init") << "\n";
  file << "gravity_align_en: " << (gravity_align_en ? "true" : "false") << "\n";
  file << "localization: " << (locate_in_prior_map ? "true" : "false") << "\n";
  file << "prior_map_update_en: " << (prior_map_update_en ? "true" : "false") << "\n";
  file << "alignment_ready: " << (initial_alignment_ready ? "true" : "false") << "\n";
  file << "T_map_camera_init: [";
  for (int i=0;i<16;++i) file << (i ? ", " : "") << initial_map_transform_(i/4,i%4);
  file << "]\n";
}

void LIVMapper::writeScanPose()
{
  if (!fout_pcd_pos.is_open()) return;
  Eigen::Matrix4d pose=Eigen::Matrix4d::Identity();
  pose.block<3,3>(0,0)=_state.rot_end;
  pose.block<3,1>(0,3)=_state.pos_end;
  if (pcd_save_type==2 && locate_in_prior_map) pose=initial_map_transform_*pose;
  const Eigen::Quaterniond q(pose.block<3,3>(0,0));
  fout_pcd_pos << std::setprecision(17) << pose(0,3) << " " << pose(1,3) << " " << pose(2,3)
               << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
  if (!fout_pcd_pos) throw std::runtime_error("Cannot write segment pose record.");
}

void LIVMapper::savePCD() 
{
  if (isPcdSavingEnabled() && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval != 0)
  {
    std::string raw_points_dir = (std::filesystem::path(pcd_output_dir) / (pcd_file_prefix + (pcd_save_interval > 0 ? "_remainder" : "") + ".pcd")).string();
    std::string downsampled_points_dir = (std::filesystem::path(pcd_output_dir) / (pcd_file_prefix + "_downsampled.pcd")).string();
    pcl::PCDWriter pcd_writer;

    if (!pcl_wait_save->empty())
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      if (pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save) != 0) throw std::runtime_error("PCD write failed."); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      if (pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud) != 0) throw std::runtime_error("PCD write failed."); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      if (pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity) != 0) throw std::runtime_error("PCD write failed.");
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
    if (pcd_save_interval>0) writeScanPose();
    writeMapMetadata();
  }
}

//FASTLIVO2的主运行循环
void LIVMapper::run(rclcpp::Node::SharedPtr &node) 
{
  rclcpp::Rate rate(5000);//设置循环频率为5000Hz
  rclcpp::WallRate initial_pose_wait_rate(100);
  std::exception_ptr run_error;
  try
  {
    while (rclcpp::ok())
    {
      // 等待期间处理初值等回调；有效初值到达后才创建传感器订阅。
      rclcpp::spin_some(this->node);
      if (!rclcpp::ok()) break;

      const bool waiting_for_initial_pose = locate_in_prior_map && !initial_pose_received;
      if (waiting_for_initial_pose)
      {
        RCLCPP_INFO_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 3000, "Waiting for initial pose...");
      }
      // 未收到初值时短路，不执行数据同步、IMU 初始化或状态估计。
      if (waiting_for_initial_pose || !sync_packages(LidarMeasures))
      {
        try
        {
          if (waiting_for_initial_pose)
          {
            initial_pose_wait_rate.sleep();
          }
          else
          {
            rate.sleep();
          }
        }
        catch (const std::runtime_error &)
        {
          // Ctrl+C 可能在进入 sleep 前关闭上下文，仍需走到循环后的 savePCD()。
          if (!rclcpp::ok()) break;
          throw;
        }
        continue;
      }

      //若同步成功则依次执行首次帧初始化（handleFirstFrame）、IMU 处理（processImu）和状态估计/建图（stateEstimationAndMapping）。
      handleFirstFrame();//对时间戳进行首次帧初始化。

      processImu();

      // if (!p_imu->imu_time_init) continue;

      stateEstimationAndMapping();
    }
  }
  catch (const rclcpp::exceptions::RCLError &)
  {
    // SIGINT can invalidate the context between ok() and wait-set creation,
    // or during a publisher call. Only shutdown-related ROS errors are benign.
    if (rclcpp::ok()) run_error = std::current_exception();
  }
  catch (...)
  {
    run_error = std::current_exception();
  }
  // Save only previously accepted observations, on normal shutdown or a caught
  // runtime failure. Preserve the original failure even if final saving fails.
  if (run_error)
    RCLCPP_WARN(this->node->get_logger(), "Mapping interrupted by an exception; attempting to save valid observations before exiting.");
  try
  {
    savePCD();
  }
  catch (const std::exception &error)
  {
    RCLCPP_ERROR(this->node->get_logger(), "Final PCD save failed: %s", error.what());
    if (!run_error) throw;
  }
  if (run_error) std::rethrow_exception(run_error);
}

//prop_imu_once 对给定的状态快照 imu_prop_state 进行一次 IMU 前向传播（单步积分）。
//输入是时间间隔 dt、平均加速度 acc_avr 和平均角速度 angvel_avr；
//函数把 IMU 读数去偏、换算到全局坐标系并更新旋转、位置与速度。
void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback()
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once ) { return; }
  /*『GT』*/
  if (locate_in_prior_map && !initial_pose_received)
  {
    return;
  }
  /*『GT』*/
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制 propagate 频率和 IMU 频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && stamp2Sec(prop_imu_buffer.front().header.stamp) < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = stamp2Sec(prop_imu_buffer[i].header.stamp) - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = stamp2Sec(newest_imu.header.stamp) - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.rot_end.transpose() * imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "camera_init";
    imu_prop_odom.child_frame_id = "aft_mapped";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom->publish(imu_prop_odom);

    V3D ang_vel_body(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
    ang_vel_body -= imu_propagate.bias_g;
    publish_state6(pubState6ImuProp, imu_propagate.rot_end, ang_vel_body);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Eigen::Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  /*『GT』*/
  if (locate_in_prior_map && !initial_pose_received)
  {
    return;
  }
  /*『GT』*/
  if (!lidar_en) return;
  mtx_buffer.lock();
  // cout<<"got feature"<<endl;
  if (stamp2Sec(msg->header.stamp) < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(),"lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
    lid_header_time_buffer.clear();
    lidar_pushed = false;
  }
  // ROS_INFO("get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  if (!ptr || ptr->size()<2) { mtx_buffer.unlock(); return; }
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(stamp2Sec(msg->header.stamp));
  last_timestamp_lidar = stamp2Sec(msg->header.stamp);

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in)
{
  if (!lidar_en) return;
  /*『GT』*/
  if (locate_in_prior_map && !initial_pose_received)
  {
    return;
  }
  /*『GT』*/
  mtx_buffer.lock();
  livox_ros_driver2::msg::CustomMsg::SharedPtr msg(new livox_ros_driver2::msg::CustomMsg(*msg_in));
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - stamp2Sec(msg->header.stamp)) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - stamp2Sec(msg->header.stamp);
    if (log_sensor) RCLCPP_INFO(this->node->get_logger(), "\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = stamp2Sec(msg->header.stamp);
  if (log_sensor) RCLCPP_INFO(this->node->get_logger(), "Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(), "lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
    lid_header_time_buffer.clear();
    lidar_pushed = false;
  }
  if (log_sensor) RCLCPP_INFO(this->node->get_logger(), "get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->size()<2) {
    RCLCPP_ERROR(this->node->get_logger(), "Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}
// A service response confirms acceptance, not completion of IMU/prior-map initialization.
void LIVMapper::set_initial_pose_cbk(
    const fast_livo::srv::SetInitialPose::Request::SharedPtr request,
    fast_livo::srv::SetInitialPose::Response::SharedPtr response)
{
  const auto reject = [&](const std::string &reason) {
    response->accepted = false;
    response->message = reason;
    RCLCPP_WARN(this->node->get_logger(), "Initial pose rejected: %s", reason.c_str());
  };
  if (!locate_in_prior_map) {
    reject("Initial poses are only accepted in prior-map localization mode.");
    return;
  }
  if (request->request_id.empty()) {
    reject("request_id must not be empty.");
    return;
  }
  if (initial_pose_received) {
    // A lost response may cause the exact same request to be sent again.
    // Never replace the accepted pose or recreate subscriptions on a retry.
    if (request->request_id == accepted_initial_pose_request_.request_id &&
        request->pose == accepted_initial_pose_request_.pose) {
      response->accepted = true;
      response->message = "This request was already accepted; initialization continues independently.";
    } else {
      reject("This mapping session already accepted an initial pose; only the identical request may be retried.");
    }
    return;
  }
  const auto &msg = request->pose;
  const auto &p = msg.pose.pose.position;
  const auto &o = msg.pose.pose.orientation;
  Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
  if (msg.header.frame_id != map_frame_id_ || !std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(p.z) || !q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-12 ||
      !std::all_of(msg.pose.covariance.begin(), msg.pose.covariance.end(), [](double v) { return std::isfinite(v); }))
  {
    reject("Expected frame '" + map_frame_id_ + "', finite position/covariance and a non-zero finite quaternion.");
    return;
  }
  auto normalized_pose = msg;
  q.normalize();
  normalized_pose.pose.pose.orientation.x = q.x();
  normalized_pose.pose.pose.orientation.y = q.y();
  normalized_pose.pose.pose.orientation.z = q.z();
  normalized_pose.pose.pose.orientation.w = q.w();
  try {
    initializeSensorSubscriptions();
  } catch (const std::exception &error) {
    sub_pcl.reset();
    sub_imu.reset();
    sub_img.reset();
    reject(std::string("Cannot create sensor subscriptions: ") + error.what());
    return;
  }
  initial_pose = normalized_pose;
  accepted_initial_pose_request_ = *request;
  initial_pose_received = true;
  response->accepted = true;
  response->message = "Initial IMU pose accepted; keep stationary until prior-map initialization completes.";
  RCLCPP_INFO(this->node->get_logger(), "%s", response->message.c_str());
}

void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
  if (!imu_en) return;
  /*『GT』*/
  if (locate_in_prior_map && !initial_pose_received)
  {
    return;
  }
  /*『GT』*/  
  if (last_timestamp_lidar < 0.0) return;
  if (log_sensor) RCLCPP_INFO(this->node->get_logger(), "get imu at time: %.6f", stamp2Sec(msg_in->header.stamp));
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  msg->header.stamp = sec2Stamp(stamp2Sec(msg->header.stamp) - imu_time_offset);
  double timestamp = stamp2Sec(msg->header.stamp);

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    RCLCPP_WARN(this->node->get_logger(), "IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = sec2Stamp(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    RCLCPP_ERROR(this->node->get_logger(), "imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  {
    RCLCPP_WARN(this->node->get_logger(), "imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  if (log_sensor) cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
  return img;
}

// static int i = 0;
//vio的用不上
void LIVMapper::img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in)
{
  if (!img_en) return;
  /*『GT』*/
  if (locate_in_prior_map && !initial_pose_received)
  {
    return;
  }
  /*『GT』*/
  sensor_msgs::msg::Image::SharedPtr msg(new sensor_msgs::msg::Image(*msg_in));
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   RCLCPP_WARN(this->node->get_logger(), "img jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  // if (hilti_en)
  // {
  //   i++;
  //   if (i % 4 != 0) return;
  // }
  // double msg_header_time =  stamp2Sec(msg->header.stamp);
  double msg_header_time = stamp2Sec(msg->header.stamp) + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  if (log_sensor) RCLCPP_INFO(this->node->get_logger(), "Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    RCLCPP_ERROR(this->node->get_logger(), "image loop back. \n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    RCLCPP_WARN(this->node->get_logger(), "Image need Jumps: %.6f", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur;
  try { img_cur = getImageFromMsg(msg); }
  catch (const cv_bridge::Exception &error) {
    mtx_buffer.unlock();
    RCLCPP_WARN(this->node->get_logger(), "Invalid image: %s", error.what());
    return;
  }
  if (img_cur.empty()) { mtx_buffer.unlock(); return; }
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = stamp2Sec(imu_buffer.back()->header.stamp);

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        RCLCPP_ERROR(this->node->get_logger(), "[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        // RCLCPP_ERROR(this->node->get_logger(), "lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty())
      {
        if (stamp2Sec(imu_buffer.front()->header.stamp) > m.lio_time) break;

        if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // stamp2Sec(imu_buffer.front()->header.stamp));
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      double imu_time = stamp2Sec(imu_buffer.front()->header.stamp);

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = stamp2Sec(imu_buffer.front()->header.stamp);
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   stamp2Sec(imu_buffer.front()->header.stamp));
      // }
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  RCLCPP_ERROR(this->node->get_logger(), "out sync");
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = this->node->get_clock()->now();
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

void LIVMapper::publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  if (img_en)
  {
    static int pub_num = 1;
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num == pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (!pf.allFinite() || pf[2] <= 1e-9) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255;
          // else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255;
          // else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255;
          // else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
    else
    {
      pub_num++;
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  if (img_en)
  {
    // cout << "RGB pointcloud size: " << laserCloudWorldRGB->size() << endl;
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  else 
  { 
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  }
  laserCloudmsg.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time); //.fromSec(last_timestamp_lidar);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes->publish(laserCloudmsg);

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (isPcdSavingEnabled())
  {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    int &scan_wait_num = save_scan_count;

    if (pcd_save_type == 1) {
      PointCloudXYZI body;
      body.reserve(feats_undistort->size());
      for (auto point : *feats_undistort) {
        const V3D p = extR * V3D(point.x,point.y,point.z) + extT;
        point.x=p.x(); point.y=p.y(); point.z=p.z(); body.push_back(point);
      }
      *pcl_wait_save_intensity += body;
    } else if (img_en) {
      if (pcd_save_type == 2 && locate_in_prior_map) {
        pcl::PointCloud<pcl::PointXYZRGB> in_map;
        pcl::transformPointCloud(*laserCloudWorldRGB, in_map, initial_map_transform_.cast<float>().eval());
        *pcl_wait_save += in_map;
      } else *pcl_wait_save += *laserCloudWorldRGB;
    } else {
      if (pcd_save_type == 2 && locate_in_prior_map) {
        PointCloudXYZI in_map;
        pcl::transformPointCloud(*pcl_w_wait_pub, in_map, initial_map_transform_.cast<float>().eval());
        *pcl_wait_save_intensity += in_map;
      } else *pcl_wait_save_intensity += *pcl_w_wait_pub;
    }
    scan_wait_num++;

    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      pcd_index++;
      std::ostringstream filename;
      filename << pcd_file_prefix << "_" << std::setfill('0') << std::setw(4) << pcd_index << ".pcd";
      string all_points_dir = (std::filesystem::path(pcd_output_dir) / filename.str()).string();
      pcl::PCDWriter pcd_writer;
      if (isPcdSavingEnabled())
      {
        cout << "current scan saved to: " << all_points_dir << endl;
        if (!pcl_wait_save->empty())
        {
          if (pcd_writer.writeBinary(all_points_dir, *pcl_wait_save) != 0) throw std::runtime_error("PCD write failed."); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
          PointCloudXYZRGB().swap(*pcl_wait_save);
        }
        else
        {
          if (pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity) != 0) throw std::runtime_error("PCD write failed.");
          PointCloudXYZI().swap(*pcl_wait_save_intensity);
        }        
        writeScanPose();
        scan_wait_num = 0;
      }
    }
  }
  if (img_save_en && img_en && !vio_manager->img_rgb.empty() && ++image_count % img_save_interval == 0) {
    const std::string name = pcd_file_prefix + "_" + std::to_string(image_count) + ".png";
    if (!cv::imwrite((std::filesystem::path(image_output_dir)/name).string(), vio_manager->img_rgb))
      throw std::runtime_error("Image save failed.");
  }
  if(laserCloudWorldRGB->size() > 0)  PointCloudXYZI().swap(*pcl_wait_pub); 
  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubSubVisualMap->publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = this->node->get_clock()->now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void LIVMapper::publish_state6(const rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr &pubState6,
                               const M3D &rot_end,
                               const V3D &base_ang_vel)
{
  if (!pubState6) return;

  const V3D gravity_world(0.0, 0.0, -1.0);
  const V3D projected_gravity = rot_end.transpose() * gravity_world;

  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(6);
  msg.data[0] = static_cast<float>(base_ang_vel(0));
  msg.data[1] = static_cast<float>(base_ang_vel(1));
  msg.data[2] = static_cast<float>(base_ang_vel(2));
  msg.data[3] = static_cast<float>(projected_gravity(0));
  msg.data[4] = static_cast<float>(projected_gravity(1));
  msg.data[5] = static_cast<float>(projected_gravity(2));
  pubState6->publish(msg);
  const bool propagated = pubState6 == pubState6ImuProp;
  auto publisher = propagated ? state6_imu_stamped_pub_ : state6_stamped_pub_;
  if (publisher) {
    fast_livo::msg::State6Stamped stamped;
    stamped.header.stamp = propagated ? rclcpp::Time(newest_imu.header.stamp) : sec2Stamp(LidarMeasures.last_lio_update_time);
    stamped.header.frame_id = "aft_mapped";
    stamped.angular_velocity.x = base_ang_vel.x(); stamped.angular_velocity.y = base_ang_vel.y(); stamped.angular_velocity.z = base_ang_vel.z();
    stamped.projected_gravity.x = projected_gravity.x(); stamped.projected_gravity.y = projected_gravity.y(); stamped.projected_gravity.z = projected_gravity.z();
    stamped.valid = tracking_valid;
    publisher->publish(stamped);
  }
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time); //.ros::Time()fromSec(last_timestamp_lidar);
  set_posestamp(odomAftMapped.pose.pose);
  const V3D velocity_body = _state.rot_end.transpose() * _state.vel_end;
  odomAftMapped.twist.twist.linear.x = velocity_body.x();
  odomAftMapped.twist.twist.linear.y = velocity_body.y();
  odomAftMapped.twist.twist.linear.z = velocity_body.z();
  odomAftMapped.twist.twist.angular.x = p_imu->unbiased_gyr.x();
  odomAftMapped.twist.twist.angular.y = p_imu->unbiased_gyr.y();
  odomAftMapped.twist.twist.angular.z = p_imu->unbiased_gyr.z();
  Eigen::Matrix<double,6,6> pose_cov, pose_jac = Eigen::Matrix<double,6,6>::Zero();
  pose_cov.topLeftCorner<3,3>() = _state.cov.block<3,3>(3,3);
  pose_cov.topRightCorner<3,3>() = _state.cov.block<3,3>(3,0);
  pose_cov.bottomLeftCorner<3,3>() = _state.cov.block<3,3>(0,3);
  pose_cov.bottomRightCorner<3,3>() = _state.cov.block<3,3>(0,0);
  pose_jac.topLeftCorner<3,3>().setIdentity();
  pose_jac.bottomRightCorner<3,3>() = _state.rot_end;
  const Eigen::Matrix<double,6,6> local_cov = pose_jac * pose_cov * pose_jac.transpose();
  for (int i=0;i<6;++i) for (int j=0;j<6;++j) odomAftMapped.pose.covariance[6*i+j] = local_cov(i,j);
  Eigen::Matrix<double,6,DIM_STATE> twist_jac=Eigen::Matrix<double,6,DIM_STATE>::Zero();
  M3D velocity_cross;
  velocity_cross << SKEW_SYM_MATRX(velocity_body);
  twist_jac.block<3,3>(0,0)=velocity_cross;
  twist_jac.block<3,3>(0,7)=_state.rot_end.transpose();
  twist_jac.block<3,3>(3,10)=-M3D::Identity();
  Eigen::Matrix<double,6,6> twist_cov=twist_jac*_state.cov*twist_jac.transpose();
  twist_cov.bottomRightCorner<3,3>()+=p_imu->cov_gyr.asDiagonal();
  for(int i=0;i<6;++i) for(int j=0;j<6;++j) odomAftMapped.twist.covariance[6*i+j]=twist_cov(i,j);


  tf2::Transform transform;
  tf2::Quaternion q;
  transform.setOrigin(tf2::Vector3(_state.pos_end(0), _state.pos_end(1), _state.pos_end(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);
  tf_broadcaster_->sendTransform(geometry_msgs::msg::TransformStamped(createTransformStamped(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped")));
  pubOdomAftMapped->publish(odomAftMapped);

  if (!tf_buffer_ || !pubOdomAftMappedInMap) {
    return;
  }

  try
  {
    const auto map_to_camera_msg = tf_buffer_->lookupTransform(map_frame_id_, "camera_init", rclcpp::Time(odomAftMapped.header.stamp));

    tf2::Transform map_to_camera;
    tf2::Quaternion q_map_to_camera(
      map_to_camera_msg.transform.rotation.x,
      map_to_camera_msg.transform.rotation.y,
      map_to_camera_msg.transform.rotation.z,
      map_to_camera_msg.transform.rotation.w);
    q_map_to_camera.normalize();
    map_to_camera.setOrigin(tf2::Vector3(
      map_to_camera_msg.transform.translation.x,
      map_to_camera_msg.transform.translation.y,
      map_to_camera_msg.transform.translation.z));
    map_to_camera.setRotation(q_map_to_camera);

    const tf2::Transform map_to_aft = map_to_camera * transform;

    odomAftMappedInMap = odomAftMapped;
    odomAftMappedInMap.header.frame_id = map_frame_id_;
    odomAftMappedInMap.pose.pose.position.x = map_to_aft.getOrigin().x();
    odomAftMappedInMap.pose.pose.position.y = map_to_aft.getOrigin().y();
    odomAftMappedInMap.pose.pose.position.z = map_to_aft.getOrigin().z();
    odomAftMappedInMap.pose.pose.orientation.x = map_to_aft.getRotation().x();
    odomAftMappedInMap.pose.pose.orientation.y = map_to_aft.getRotation().y();
    odomAftMappedInMap.pose.pose.orientation.z = map_to_aft.getRotation().z();
    odomAftMappedInMap.pose.pose.orientation.w = map_to_aft.getRotation().w();

    const auto &qr = map_to_camera_msg.transform.rotation;
    const M3D map_rotation = Eigen::Quaterniond(qr.w,qr.x,qr.y,qr.z).normalized().toRotationMatrix();
    Eigen::Matrix<double,6,6> covariance_rotation = Eigen::Matrix<double,6,6>::Zero();
    covariance_rotation.topLeftCorner<3,3>() = map_rotation;
    covariance_rotation.bottomRightCorner<3,3>() = map_rotation;
    const Eigen::Matrix<double,6,6> map_cov = covariance_rotation * local_cov * covariance_rotation.transpose();
    for (int i=0;i<6;++i) for (int j=0;j<6;++j) odomAftMappedInMap.pose.covariance[6*i+j] = map_cov(i,j);
    pubOdomAftMappedInMap->publish(odomAftMappedInMap);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_WARN_THROTTLE(
      this->node->get_logger(),
      *this->node->get_clock(),
      5000,
      "No map->camera_init TF yet, skip publishing /aft_mapped_in_map: %s",
      ex.what());
  }
}

void LIVMapper::publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void LIVMapper::publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = sec2Stamp(LidarMeasures.last_lio_update_time);
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
}
