#include <chrono>
#include <algorithm>
#include <filesystem>
#include <omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// ROS2 Headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <fast_livo/srv/set_initial_pose.hpp>
#include <fast_livo/workspace_root.hpp>
#include <unistd.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>

// PCL Headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

// TEASER++ Headers
#include <teaser/registration.h>
#include <teaser/matcher.h>
#include <teaser/fpfh.h>

// small_gicp Headers
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>

// Livox Support
#ifdef USE_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

// 定位状态枚举
enum class LocalizationState {
    TEASER_PHASE,   // TEASER 全局定位阶段
    GICP_PHASE      // GICP 精细配准阶段
};

class TeaserGicpNode : public rclcpp::Node
{
public:
    explicit TeaserGicpNode(const std::string &algorithm)
        : Node("teaser_gicp_node"), state_(LocalizationState::TEASER_PHASE)
    {
        // Direct mapping still owns this node, but has no registration work.
        // Return before prior-map validation, subscriptions and watchdog setup.
        if (!this->declare_parameter<bool>("locate_in_prior_map", true)) {
            RCLCPP_INFO(this->get_logger(),
                "Direct mapping: registration disabled; transform_publisher provides identity map alignment.");
            return;
        }
        // ================= 参数声明 =================
        
        this->declare_parameter("algorithm", algorithm);
        this->declare_parameter("max_attempts", 300);
        this->declare_parameter("timeout_seconds", 120.0);
        this->declare_parameter("initial_pose_timeout_seconds", 30.0);
        algorithm_ = this->get_parameter("algorithm").as_string();
        max_attempts_ = this->get_parameter("max_attempts").as_int();
        timeout_seconds_ = this->get_parameter("timeout_seconds").as_double();
        initial_pose_timeout_seconds_ = this->get_parameter("initial_pose_timeout_seconds").as_double();
        if (algorithm_!="teaser_gicp" && algorithm_!="teaser" && algorithm_!="gicp" && algorithm_!="icp" && algorithm_!="sac_ia_icp")
            throw std::invalid_argument("algorithm must be teaser_gicp, teaser, gicp, icp or sac_ia_icp.");
        // 1. 基础配置
        this->declare_parameter("prior_map_path", "");
        this->declare_parameter("prior_map_name", "");
        this->declare_parameter("map_frame_id", "map");
        this->declare_parameter("pcl_type", "livox");
        this->declare_parameter<std::vector<double>>("extrin_calib.extrinsic_R", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("extrin_calib.extrinsic_T", std::vector<double>{});

        // 2. 降采样参数
        this->declare_parameter("map_voxel_leaf_size", 0.4);     // TEASER 用较大体素
        this->declare_parameter("cloud_voxel_leaf_size", 0.4);   // TEASER 用较大体素
        this->declare_parameter("gicp_map_voxel_leaf_size", 0.1);  // GICP 用较小体素
        this->declare_parameter("gicp_cloud_voxel_leaf_size", 0.1);

        // 3. TEASER 参数
        this->declare_parameter("fpfh_normal_radius", 0.6);
        this->declare_parameter("fpfh_feature_radius", 0.9);
        this->declare_parameter("noise_bound", 0.2);
        this->declare_parameter("teaser_solver_max_iter", 100);
        this->declare_parameter("rotation_gnc_factor", 1.4);
        this->declare_parameter("teaser_inlier_threshold", 30);    // TEASER 内点阈值
        this->declare_parameter("teaser_success_count", 3);        // TEASER 连续成功次数后切换

        // 4. GICP 参数
        this->declare_parameter("gicp_solver_max_iter", 50);
        this->declare_parameter("num_threads", 4);
        this->declare_parameter("max_correspondence_distance", 0.5);
        this->declare_parameter("fitness_score_thre", 0.1);
        this->declare_parameter("converged_count_thre", 20);
        this->declare_parameter("registration_type", "VGICP");

        // ================= 获取参数 =================
        this->get_parameter("prior_map_path", prior_map_path_);
        this->get_parameter("map_frame_id", map_frame_id_);
        this->get_parameter("pcl_type", pcl_type_);
        
        this->get_parameter("map_voxel_leaf_size", teaser_map_voxel_);
        this->get_parameter("cloud_voxel_leaf_size", teaser_cloud_voxel_);
        this->get_parameter("gicp_map_voxel_leaf_size", gicp_map_voxel_);
        this->get_parameter("gicp_cloud_voxel_leaf_size", gicp_cloud_voxel_);

        this->get_parameter("fpfh_normal_radius", fpfh_normal_radius_);
        this->get_parameter("fpfh_feature_radius", fpfh_feature_radius_);
        this->get_parameter("noise_bound", noise_bound_);
        this->get_parameter("teaser_solver_max_iter", teaser_solver_max_iter_);
        this->get_parameter("rotation_gnc_factor", rotation_gnc_factor_);
        int inlier_thr;
        this->get_parameter("teaser_inlier_threshold", inlier_thr);
        teaser_inlier_threshold_ = static_cast<size_t>(inlier_thr);
        this->get_parameter("teaser_success_count", teaser_success_count_thre_);

        this->get_parameter("gicp_solver_max_iter", gicp_solver_max_iter_);
        this->get_parameter("num_threads", num_threads_);
        this->get_parameter("max_correspondence_distance", max_correspondence_distance_);
        this->get_parameter("fitness_score_thre", fitness_score_thre_);
        this->get_parameter("converged_count_thre", gicp_converged_count_thre_);
        this->get_parameter("registration_type", registration_type_);

        // ================= 初始化 ROS 接口 =================
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("icp_result", rclcpp::QoS(1).reliable().transient_local());
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "prior_map", rclcpp::QoS(1).transient_local().reliable());
        transformed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud", 10);

        const auto configured_root = this->declare_parameter<std::string>("workspace_root", "auto");
        const auto workspace_root = fast_livo::resolveWorkspaceRoot(configured_root, FAST_LIVO2_SOURCE_DIR);
        const auto root_result = this->set_parameter(rclcpp::Parameter("workspace_root", workspace_root.string()));
        if (!root_result.successful)
            throw std::runtime_error("Cannot publish resolved workspace_root parameter: " + root_result.reason);
        RCLCPP_INFO(this->get_logger(), "workspace_root (%s): %s",
                    configured_root == "auto" ? "auto" : "manual", workspace_root.c_str());
        std::filesystem::path directory(prior_map_path_);
        if (!directory.empty() && !directory.is_absolute()) directory = workspace_root / directory;
        const std::filesystem::path filename(this->get_parameter("prior_map_name").as_string());
        if (!directory.is_absolute() || filename.empty() || filename.has_parent_path() ||
            filename == "." || filename == "..") {
            throw std::invalid_argument(
                "Prior localization requires an absolute prior_map_path directory (or a path relative to workspace_root) and a plain prior_map_name filename.");
        }
        prior_map_path_ = (directory / filename).lexically_normal().string();
        initialize_pose_conversion();
        const double positive[] = {teaser_map_voxel_, teaser_cloud_voxel_, gicp_map_voxel_, gicp_cloud_voxel_,
            fpfh_normal_radius_, fpfh_feature_radius_, noise_bound_, max_correspondence_distance_, fitness_score_thre_, timeout_seconds_};
        for (double v : positive) if (!std::isfinite(v) || v<=0) throw std::invalid_argument("Registration sizes, thresholds and timeout must be finite and positive.");
        if (num_threads_<1 || gicp_solver_max_iter_<1 || teaser_solver_max_iter_<1 || teaser_success_count_thre_<1 ||
            gicp_converged_count_thre_<1 || max_attempts_<1 || inlier_thr<3 || !std::isfinite(initial_pose_timeout_seconds_) || initial_pose_timeout_seconds_<=0 ||
            !std::isfinite(rotation_gnc_factor_) || rotation_gnc_factor_<=1 || fpfh_feature_radius_<=fpfh_normal_radius_ ||
            (registration_type_!="GICP" && registration_type_!="VGICP"))
            throw std::invalid_argument("Invalid registration count, thread count, feature radii or registration_type.");
        omp_set_num_threads(num_threads_);
        initial_pose_client_ = this->create_client<fast_livo::srv::SetInitialPose>("/fast_livo2/set_initial_pose");
        start_time_ = std::chrono::steady_clock::now();
        watchdog_ = this->create_wall_timer(std::chrono::milliseconds(100), [this] {
            if (finished_) {
                deliver_initial_pose();
            } else if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count() >= timeout_seconds_) {
                throw std::runtime_error("Relocalization timed out; no valid registration result was computed.");
            }
        });

#ifdef USE_LIVOX
        if (pcl_type_ == "livox") {
            lvx_cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                "/livox/lidar", rclcpp::SensorDataQoS().keep_last(1), std::bind(&TeaserGicpNode::lvx_cloud_callback, this, std::placeholders::_1));
        } else {
            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/pointcloud2", rclcpp::SensorDataQoS().keep_last(1), std::bind(&TeaserGicpNode::cloud_callback, this, std::placeholders::_1));
        }
#else
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud2", rclcpp::SensorDataQoS().keep_last(1), std::bind(&TeaserGicpNode::cloud_callback, this, std::placeholders::_1));
#endif
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&TeaserGicpNode::pose_callback, this, std::placeholders::_1));

        // ================= 加载地图 =================
        if (!load_map()) {
            throw std::runtime_error("Failed to load prior map: " + prior_map_path_);
        }

        // ================= 初始化 TEASER =================
        if (algorithm_ == "teaser_gicp" || algorithm_ == "teaser") init_teaser();

        // ================= 初始化 GICP =================
        init_gicp();
        if (algorithm_ == "gicp" || algorithm_ == "icp") state_ = LocalizationState::GICP_PHASE;

        RCLCPP_INFO(this->get_logger(), "=== TeaserGicpNode Initialized ===");
        RCLCPP_INFO(this->get_logger(), "Starting in TEASER phase for global localization...");
    }

private:
    void initialize_pose_conversion()
    {
        const auto rotation = this->get_parameter("extrin_calib.extrinsic_R").as_double_array();
        const auto translation = this->get_parameter("extrin_calib.extrinsic_T").as_double_array();
        if (rotation.size() != 9 || translation.size() != 3) {
            throw std::invalid_argument(
                "Prior-map localization requires extrin_calib.extrinsic_R (9 values, row-major) "
                "and extrin_calib.extrinsic_T (3 values) from the mapping configuration.");
        }

        Eigen::Matrix4d T_imu_lidar = Eigen::Matrix4d::Identity();
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                T_imu_lidar(row, col) = rotation[row * 3 + col];
            }
            T_imu_lidar(row, 3) = translation[row];
        }
        if (!T_imu_lidar.allFinite()) {
            throw std::invalid_argument("LiDAR-to-IMU extrinsic_R and extrinsic_T must contain only finite values.");
        }
        const Eigen::Matrix3d R_imu_lidar = T_imu_lidar.block<3, 3>(0, 0);
        constexpr double rotation_tolerance = 1e-6;
        if ((R_imu_lidar.transpose() * R_imu_lidar - Eigen::Matrix3d::Identity()).norm() > rotation_tolerance ||
            std::abs(R_imu_lidar.determinant() - 1.0) > rotation_tolerance) {
            throw std::invalid_argument("extrin_calib.extrinsic_R must be an orthonormal rotation matrix with determinant +1.");
        }
        T_lidar_imu_ = T_imu_lidar.inverse();
        RCLCPP_INFO(this->get_logger(),
            "/icp_result represents the initial IMU pose in map: T_map_lidar * inverse(T_imu_lidar). "
            "FAST-LIVO2 applies its measured gravity rotation when enabled. "
            "Keep the device stationary until prior-map initialization completes.");
    }

    // ===================== 地图加载 =====================
    bool load_map()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(prior_map_path_, *raw_map) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Could not read map file: %s", prior_map_path_.c_str());
            return false;
        }
        if (std::any_of(raw_map->begin(), raw_map->end(), [](const auto &point){ return !pcl::isFinite(point); }))
            throw std::invalid_argument("Prior map contains non-finite XYZ.");
        if (raw_map->empty()) {
            RCLCPP_ERROR(this->get_logger(), "Prior map contains no points: %s", prior_map_path_.c_str());
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded map with %zu points", raw_map->size());

        // 为 TEASER 降采样地图 (较大体素)
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(raw_map);
        sor.setLeafSize(teaser_map_voxel_, teaser_map_voxel_, teaser_map_voxel_);
        sor.filter(*teaser_map_pcl_);
        RCLCPP_INFO(this->get_logger(), "TEASER map downsampled to %zu points", teaser_map_pcl_->size());

        // 为 GICP 降采样地图 (较小体素)
        gicp_map_pcl_ = small_gicp::voxelgrid_sampling_omp(*raw_map, gicp_map_voxel_, num_threads_);
        RCLCPP_INFO(this->get_logger(), "GICP map downsampled to %zu points", gicp_map_pcl_->size());

        if (teaser_map_pcl_->size()<10 || gicp_map_pcl_->size()<10)
            throw std::invalid_argument("Prior map has fewer than 10 points after registration downsampling.");
        convert_pcl_to_teaser(teaser_map_pcl_, teaser_map_cloud_);
        // 发布地图
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*gicp_map_pcl_, map_msg);  // 发布更精细的地图用于可视化
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = map_frame_id_;
        map_pub_->publish(map_msg);

        return true;
    }

    // ===================== TEASER 初始化 =====================
    void init_teaser()
    {
        // 转换为 TEASER 格式
        convert_pcl_to_teaser(teaser_map_pcl_, teaser_map_cloud_);

        // 计算地图 FPFH
        RCLCPP_INFO(this->get_logger(), "Computing Map FPFH features...");
        teaser::FPFHEstimation fpfh;
        teaser_map_descriptors_ = fpfh.computeFPFHFeatures(teaser_map_cloud_, fpfh_normal_radius_, fpfh_feature_radius_);
        RCLCPP_INFO(this->get_logger(), "Map FPFH features computed.");
    }

    // ===================== GICP 初始化 =====================
    void init_gicp()
    {
        gicp_reg_.setRegistrationType(registration_type_);
        gicp_reg_.setNumThreads(num_threads_);
        gicp_reg_.setMaximumIterations(gicp_solver_max_iter_);
        gicp_reg_.setMaxCorrespondenceDistance(max_correspondence_distance_);
        gicp_reg_.setCorrespondenceRandomness(20);
        gicp_reg_.setInputTarget(gicp_map_pcl_);
        
        gicp_init_guess_ = Eigen::Matrix4f::Identity();
    }

    // ===================== 核心处理函数 =====================
    void process_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        if (finished_) return;
        if (teaser_map_cloud_.empty() || gicp_map_pcl_->empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Map not ready.");
            return;
        }

        if (!input_cloud || input_cloud->size()<10) return;
        input_cloud->erase(std::remove_if(input_cloud->begin(), input_cloud->end(), [](const auto &point){ return !pcl::isFinite(point); }), input_cloud->end());
        if (input_cloud->size()<10) return;
        if (++attempts_ > max_attempts_ || std::chrono::duration<double>(std::chrono::steady_clock::now()-start_time_).count()>=timeout_seconds_)
            throw std::runtime_error("Relocalization retry/time limit reached without a valid result.");
        if (algorithm_=="sac_ia_icp" && state_==LocalizationState::TEASER_PHASE) { process_sac(input_cloud); return; }
        switch (state_) {
            case LocalizationState::TEASER_PHASE:
                process_teaser(input_cloud);
                break;
            case LocalizationState::GICP_PHASE:
                process_gicp(input_cloud);
                break;
        }
    }

    // ===================== TEASER 处理 =====================
    void process_teaser(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        // 1. 降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(input_cloud);
        sor.setLeafSize(teaser_cloud_voxel_, teaser_cloud_voxel_, teaser_cloud_voxel_);
        sor.filter(*src_cloud);

        if (src_cloud->size() < 50) {
            RCLCPP_WARN(this->get_logger(), "[TEASER] Input cloud too small: %zu", src_cloud->size());
            return;
        }

        // 2. 计算 FPFH
        teaser::PointCloud src_teaser;
        convert_pcl_to_teaser(src_cloud, src_teaser);
        teaser::FPFHEstimation fpfh;
        auto src_descriptors = fpfh.computeFPFHFeatures(src_teaser, fpfh_normal_radius_, fpfh_feature_radius_);

        // 3. 特征匹配
        teaser::Matcher matcher;
        auto correspondences = matcher.calculateCorrespondences(
            src_teaser, teaser_map_cloud_, *src_descriptors, *teaser_map_descriptors_, 
            false, true, false, 0.95);

        RCLCPP_INFO(this->get_logger(), "[TEASER] Correspondences found: %zu (threshold: %zu)", 
            correspondences.size(), teaser_inlier_threshold_);

        if (correspondences.size() < std::max<size_t>(3, teaser_inlier_threshold_)) {  // 至少需要一些对应点才能求解
            RCLCPP_WARN(this->get_logger(), "[TEASER] Too few correspondences for solving");
            teaser_success_count_ = 0;
            // 发布原始点云用于调试
            publish_debug_cloud(*src_cloud);
            return;
        }

        // 4. TEASER 求解
        teaser::RobustRegistrationSolver::Params params;
        params.noise_bound = noise_bound_;
        params.cbar2 = 1;
        params.estimate_scaling = false;
        params.rotation_max_iterations = teaser_solver_max_iter_;
        params.rotation_gnc_factor = rotation_gnc_factor_;
        params.rotation_estimation_algorithm =
            teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
        params.rotation_cost_threshold = 0.005;

        teaser::RobustRegistrationSolver solver(params);
        auto start = std::chrono::high_resolution_clock::now();
        solver.solve(src_teaser, teaser_map_cloud_, correspondences);
        auto end = std::chrono::high_resolution_clock::now();

        auto solution = solver.getSolution();

        // 5. 结果验证
        if (solution.valid && solution.rotation.allFinite() && solution.translation.allFinite()) {
            auto inliers = solver.getTranslationInliers();
            RCLCPP_INFO(this->get_logger(), "[TEASER] Solved in %.2f ms. Inliers: %zu/%zu (threshold: %zu)", 
                std::chrono::duration<double, std::milli>(end - start).count(), 
                inliers.size(), correspondences.size(), teaser_inlier_threshold_);

            // 构建变换矩阵
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform.block<3, 3>(0, 0) = solution.rotation.cast<float>();
            transform.block<3, 1>(0, 3) = solution.translation.cast<float>();

            // 发布变换后的点云用于可视化（无论是否达到阈值）
            pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
            pcl::transformPointCloud(*src_cloud, transformed_cloud, transform);
            publish_debug_cloud(transformed_cloud);

            if (inliers.size() >= teaser_inlier_threshold_) {
                teaser_success_count_++;
                RCLCPP_INFO(this->get_logger(), "[TEASER] Success count: %d / %d", 
                    teaser_success_count_, teaser_success_count_thre_);

                // 更新 GICP 初值
                gicp_init_guess_ = transform;

                // 检查是否达到切换条件
                if (teaser_success_count_ >= teaser_success_count_thre_) {
                    RCLCPP_INFO(this->get_logger(), "====================================");
                    RCLCPP_INFO(this->get_logger(), "TEASER phase complete! Switching to GICP phase...");
                    RCLCPP_INFO(this->get_logger(), "Initial pose from TEASER: x=%.3f, y=%.3f, z=%.3f", 
                        transform(0, 3), transform(1, 3), transform(2, 3));
                    RCLCPP_INFO(this->get_logger(), "====================================");
                    
                    if (algorithm_=="teaser") { complete(transform, transformed_cloud); return; }
                    state_ = LocalizationState::GICP_PHASE;
                    gicp_converged_count_ = 0;
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "[TEASER] Inliers (%zu) below threshold (%zu), resetting count",
                    inliers.size(), teaser_inlier_threshold_);
                teaser_success_count_ = 0;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "[TEASER] Solution invalid.");
            teaser_success_count_ = 0;
            // 发布原始点云
            publish_debug_cloud(*src_cloud);
        }
    }

    // ===================== GICP 处理 =====================
    void process_gicp(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        // 1. 降采样
        auto src_cloud = small_gicp::voxelgrid_sampling_omp(*input_cloud, gicp_cloud_voxel_, num_threads_);

        if (src_cloud->size()<10) return;
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
        bool converged;
        double fitness_score;
        Eigen::Matrix4f final_transform;
        if (algorithm_=="icp" || algorithm_=="sac_ia_icp") {
            pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
            icp.setMaximumIterations(gicp_solver_max_iter_);
            icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setInputSource(src_cloud); icp.setInputTarget(gicp_map_pcl_);
            icp.align(aligned_cloud,gicp_init_guess_);
            converged=icp.hasConverged(); fitness_score=icp.getFitnessScore(); final_transform=icp.getFinalTransformation();
        } else {
            gicp_reg_.setInputSource(src_cloud);
            gicp_reg_.align(aligned_cloud, gicp_init_guess_);
            converged=gicp_reg_.hasConverged(); fitness_score=gicp_reg_.getFitnessScore(); final_transform=gicp_reg_.getFinalTransformation();
        }
        converged = converged && std::isfinite(fitness_score) && final_transform.allFinite();
        RCLCPP_INFO(this->get_logger(), "[GICP] Fitness: %.4f, Converged: %d", fitness_score, converged);

        if (converged && fitness_score < fitness_score_thre_) {
            gicp_converged_count_++;
            RCLCPP_INFO(this->get_logger(), "[GICP] Converged count: %d / %d", 
                gicp_converged_count_, gicp_converged_count_thre_);

            if (gicp_converged_count_ >= gicp_converged_count_thre_) {
                RCLCPP_INFO(this->get_logger(), "====================================");
                RCLCPP_INFO(this->get_logger(), "LOCALIZATION SUCCESS!");
                RCLCPP_INFO(this->get_logger(), "Final LiDAR pose in map: x=%.3f, y=%.3f, z=%.3f",
                    final_transform(0, 3), final_transform(1, 3), final_transform(2, 3));
                RCLCPP_INFO(this->get_logger(), "====================================");

                complete(final_transform, aligned_cloud);
                return;
            } else {
                // 更新初值
                gicp_init_guess_ = final_transform;
            }
        } else {
            if (converged && fitness_score >= fitness_score_thre_) {
                // 收敛但分数高，可能是局部最优，仍然更新
                gicp_init_guess_ = final_transform;
                RCLCPP_WARN(this->get_logger(), "[GICP] High fitness score (%.3f > %.3f), may need better init",
                    fitness_score, fitness_score_thre_);
                
                // 如果连续多次 fitness score 过高，考虑回退到 TEASER
                gicp_fail_count_++;
                if (gicp_fail_count_ > 10) {
                    RCLCPP_WARN(this->get_logger(), "GICP failing too often, switching back to TEASER...");
                    state_ = (algorithm_=="gicp" || algorithm_=="icp") ? LocalizationState::GICP_PHASE : LocalizationState::TEASER_PHASE;
                    teaser_success_count_ = 0;
                    gicp_fail_count_ = 0;
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "[GICP] Not converged, resetting...");
                gicp_fail_count_++;
                
                if (gicp_fail_count_ > 5) {
                    RCLCPP_WARN(this->get_logger(), "GICP failing, switching back to TEASER...");
                    state_ = (algorithm_=="gicp" || algorithm_=="icp") ? LocalizationState::GICP_PHASE : LocalizationState::TEASER_PHASE;
                    teaser_success_count_ = 0;
                    gicp_fail_count_ = 0;
                }
            }
            gicp_converged_count_ = 0;
        }

        // 发布调试点云
        pcl::PointCloud<pcl::PointXYZ> debug_cloud;
        pcl::transformPointCloud(*src_cloud, debug_cloud, gicp_init_guess_);
        publish_debug_cloud(debug_cloud);
    }

    void complete(const Eigen::Matrix4f &transform, const pcl::PointCloud<pcl::PointXYZ> &cloud)
    {
        publish_pose(transform);
        publish_debug_cloud(cloud);
        finished_ = true;
        delivery_start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(),
            "Registration succeeded; waiting up to %.1f seconds for mapping to accept the initial pose via service. Keep stationary.",
            initial_pose_timeout_seconds_);
        deliver_initial_pose();
    }

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> normal_estimation(num_threads_);
        normal_estimation.setInputCloud(cloud); normal_estimation.setRadiusSearch(fpfh_normal_radius_); normal_estimation.compute(*normals);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr output(new pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::FPFHEstimationOMP<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> estimation;
        estimation.setNumberOfThreads(num_threads_); estimation.setInputCloud(cloud); estimation.setInputNormals(normals);
        estimation.setRadiusSearch(fpfh_feature_radius_); estimation.compute(*output);
        return output;
    }

    void process_sac(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input)
    {
        auto source=small_gicp::voxelgrid_sampling_omp(*input,teaser_cloud_voxel_,num_threads_);
        if (source->size()<10) return;
        if (!sac_target_features_) sac_target_features_=features(teaser_map_pcl_);
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ,pcl::PointXYZ,pcl::FPFHSignature33> sac;
        sac.setInputSource(source); sac.setSourceFeatures(features(source));
        sac.setInputTarget(teaser_map_pcl_); sac.setTargetFeatures(sac_target_features_);
        sac.setMaximumIterations(teaser_solver_max_iter_); sac.setMinSampleDistance(noise_bound_);
        sac.setCorrespondenceRandomness(std::min<int>(20,teaser_map_pcl_->size()));
        pcl::PointCloud<pcl::PointXYZ> aligned;
        sac.align(aligned);
        if (!sac.hasConverged() || !sac.getFinalTransformation().allFinite()) return;
        gicp_init_guess_=sac.getFinalTransformation();
        state_=LocalizationState::GICP_PHASE;
        gicp_converged_count_=0;
    }

    // ===================== 辅助函数 =====================
    void convert_pcl_to_teaser(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, teaser::PointCloud& teaser_cloud)
    {
        teaser_cloud.clear();
        teaser_cloud.reserve(pcl_cloud->size());
        for (const auto& p : *pcl_cloud) {
            teaser_cloud.push_back({p.x, p.y, p.z});
        }
    }

    void publish_pose(const Eigen::Matrix4f& transform)
    {
        // Registration and debug clouds keep T_map_lidar. Only the initial-pose
        // interface changes to the initial IMU frame; gravity alignment belongs to mapping.
        const Eigen::Matrix4d T_map_initial_imu = transform.cast<double>() * T_lidar_imu_;
        if (!T_map_initial_imu.allFinite()) {
            throw std::runtime_error("Cannot publish initial pose: converted map-to-initial-IMU transform is not finite.");
        }
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = map_frame_id_;

        pose_msg.pose.pose.position.x = T_map_initial_imu(0, 3);
        pose_msg.pose.pose.position.y = T_map_initial_imu(1, 3);
        pose_msg.pose.pose.position.z = T_map_initial_imu(2, 3);

        Eigen::Quaterniond q(T_map_initial_imu.block<3, 3>(0, 0));
        q.normalize();
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();

        for (int i = 0; i < 36; i++) pose_msg.pose.covariance[i] = 0.0;
        pose_msg.pose.covariance[0] = 0.01;
        pose_msg.pose.covariance[7] = 0.01;
        pose_msg.pose.covariance[35] = 0.01;

        initial_pose_request_ = std::make_shared<fast_livo::srv::SetInitialPose::Request>();
        initial_pose_request_->request_id = std::to_string(::getpid()) + "-" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
        initial_pose_request_->pose = pose_msg;
        // Diagnostic topic only; mapping accepts initial poses exclusively through the service.
        publisher_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(),
            "Published diagnostic initial IMU pose in map: x=%.6f, y=%.6f, z=%.6f",
            pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z);
    }

    void deliver_initial_pose()
    {
        if (initial_pose_accepted_) return;
        const auto now = std::chrono::steady_clock::now();
        if (initial_pose_future_.valid()) {
            // Poll without spinning or blocking inside the executor's timer callback.
            if (initial_pose_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                const auto response = initial_pose_future_.get();
                if (!response->accepted)
                    throw std::runtime_error("Mapping rejected the initial pose: " + response->message);
                initial_pose_accepted_ = true;
                watchdog_->cancel();
                cloud_sub_.reset();
#ifdef USE_LIVOX
                lvx_cloud_sub_.reset();
#endif
                pose_sub_.reset();
                initial_pose_future_ = {};
                initial_pose_request_.reset();
                initial_pose_client_.reset();
                RCLCPP_INFO(get_logger(),
                    "Mapping confirmed initial pose acceptance: %s Registration complete; node remains idle until shutdown. IMU/prior-map initialization continues independently.",
                    response->message.c_str());
                return;
            }
            if (now - request_sent_time_ < std::chrono::seconds(2) &&
                std::chrono::duration<double>(now - delivery_start_time_).count() < initial_pose_timeout_seconds_)
                return;
            initial_pose_client_->remove_pending_request(pending_request_id_);
            initial_pose_future_ = {};
            RCLCPP_WARN(get_logger(), "Initial-pose response not received; retrying the identical request.");
        }
        if (std::chrono::duration<double>(now - delivery_start_time_).count() >= initial_pose_timeout_seconds_)
            throw std::runtime_error("Initial pose acceptance timed out; mapping acceptance is unconfirmed.");
        if (!initial_pose_client_->service_is_ready()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "Waiting for /fast_livo2/set_initial_pose; mapping acceptance is unconfirmed.");
            return;
        }
        auto pending = initial_pose_client_->async_send_request(initial_pose_request_);
        // Keep only one pending call, with the same application request ID and pose on every retry.
        pending_request_id_ = pending.request_id;
        initial_pose_future_ = std::move(pending.future);
        request_sent_time_ = now;
    }

    void publish_debug_cloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = map_frame_id_;
        transformed_cloud_pub_->publish(cloud_msg);
    }

    // ===================== ROS 回调 =====================
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (finished_) return;
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);
        process_cloud(input_cloud);
    }

#ifdef USE_LIVOX
    void lvx_cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        if (finished_) return;
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        input_cloud->reserve(msg->point_num);
        for (uint i = 0; i < msg->point_num; i++) {
            if (std::abs(msg->points[i].x) < 0.01 && std::abs(msg->points[i].y) < 0.01 && std::abs(msg->points[i].z) < 0.01) 
                continue;
            input_cloud->push_back(pcl::PointXYZ(msg->points[i].x, msg->points[i].y, msg->points[i].z));
        }
        process_cloud(input_cloud);
    }
#endif

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (finished_) return;
        const auto &position=msg->pose.pose.position; const auto &orientation=msg->pose.pose.orientation;
        const double norm=orientation.x*orientation.x+orientation.y*orientation.y+orientation.z*orientation.z+orientation.w*orientation.w;
        if (msg->header.frame_id!=map_frame_id_ || !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) || !std::isfinite(norm) || norm<1e-12) {
            RCLCPP_WARN(get_logger(), "Ignoring invalid manual LiDAR pose."); return;
        }
        // 手动设置初始位姿，直接跳转到 GICP 阶段
        gicp_init_guess_ = Eigen::Matrix4f::Identity();
        gicp_init_guess_(0, 3) = msg->pose.pose.position.x;
        gicp_init_guess_(1, 3) = msg->pose.pose.position.y;
        gicp_init_guess_(2, 3) = msg->pose.pose.position.z;

        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 rot_mat(q);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                gicp_init_guess_(i, j) = rot_mat[i][j];
            }
        }

        double r, p, yaw;
        rot_mat.getRPY(r, p, yaw);
        RCLCPP_INFO(this->get_logger(), "Manual pose received: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f",
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, yaw);
        RCLCPP_INFO(this->get_logger(), "Switching to GICP phase with manual initial pose...");

        state_ = LocalizationState::GICP_PHASE;
        gicp_converged_count_ = 0;
        gicp_fail_count_ = 0;
    }

    // ===================== 成员变量 =====================
    
    // ROS 接口
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
#ifdef USE_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lvx_cloud_sub_;
#endif
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Client<fast_livo::srv::SetInitialPose>::SharedPtr initial_pose_client_;
    fast_livo::srv::SetInitialPose::Request::SharedPtr initial_pose_request_;
    rclcpp::Client<fast_livo::srv::SetInitialPose>::Future initial_pose_future_;
    int64_t pending_request_id_ = 0;

    std::string algorithm_;
    int max_attempts_=300, attempts_=0;
    double timeout_seconds_=120, initial_pose_timeout_seconds_=30;
    bool finished_=false;
    bool initial_pose_accepted_=false;
    std::chrono::steady_clock::time_point start_time_, delivery_start_time_, request_sent_time_;
    rclcpp::TimerBase::SharedPtr watchdog_;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sac_target_features_;
    // 状态
    LocalizationState state_;

    // TEASER 数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr teaser_map_pcl_{new pcl::PointCloud<pcl::PointXYZ>};
    teaser::PointCloud teaser_map_cloud_;
    std::shared_ptr<teaser::FPFHCloud> teaser_map_descriptors_;
    int teaser_success_count_ = 0;
    int teaser_success_count_thre_;

    // GICP 数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr gicp_map_pcl_{new pcl::PointCloud<pcl::PointXYZ>};
    small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> gicp_reg_;
    Eigen::Matrix4f gicp_init_guess_;
    Eigen::Matrix4d T_lidar_imu_ = Eigen::Matrix4d::Identity();
    int gicp_converged_count_ = 0;
    int gicp_converged_count_thre_;
    int gicp_fail_count_ = 0;

    // 参数
    std::string prior_map_path_, map_frame_id_, pcl_type_;
    double teaser_map_voxel_, teaser_cloud_voxel_;
    double gicp_map_voxel_, gicp_cloud_voxel_;
    double fpfh_normal_radius_, fpfh_feature_radius_, noise_bound_, rotation_gnc_factor_;
    int teaser_solver_max_iter_;
    size_t teaser_inlier_threshold_;
    int gicp_solver_max_iter_, num_threads_;
    double max_correspondence_distance_, fitness_score_thre_;
    std::string registration_type_;
};

int run_registration_node(int argc, char *argv[], const char *algorithm)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<TeaserGicpNode>(algorithm));
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("teaser_gicp_node"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
