#include "velodyne_slam/ndt_matcher.hpp"
#include "velodyne_slam/initial_pose.hpp"
#include "velodyne_slam/localization_stability.hpp"
#include <future>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <deque>
#include <sstream>

namespace velodyne_slam
{
constexpr double kDegreeToRadian = 0.017453292519943295769;

struct AdaptiveNdtSettings
{
  NdtOptions options;
  double search_xy = 0.0;
  double search_yaw = 0.0;
  double local_map_radius = 0.0;
  int level = 0;
};

static Eigen::Matrix4f poseMatrix(const geometry_msgs::msg::Pose & p)
{
  Eigen::Quaternionf q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (!q.coeffs().allFinite() || q.norm() < 1e-6) {throw std::runtime_error("Invalid quaternion");}
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  m.block<3, 1>(0, 3) = Eigen::Vector3f(p.position.x, p.position.y, p.position.z);
  if (!m.allFinite()) {throw std::runtime_error("Nonfinite pose");}
  return m;
}
static geometry_msgs::msg::Pose toPose(const Eigen::Matrix4f & m)
{
  geometry_msgs::msg::Pose p;
  p.position.x = m(0, 3); p.position.y = m(1, 3); p.position.z = m(2, 3);
  Eigen::Quaternionf q(m.block<3, 3>(0, 0)); q.normalize();
  p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

class NdtLocalization : public rclcpp::Node
{
public:
  NdtLocalization()
  : Node("ndt_localization")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "imu_link");
    const auto rotation = declare_parameter<std::vector<double>>(
      "lidar_rotation_in_imu", {1., 0., 0., 0., 1., 0., 0., 0., 1.});
    if (rotation.size() != 9) {throw std::runtime_error("Expected 9 extrinsic rotation values");}
    for (int i = 0; i < 9; ++i) {imu_R_lidar_(i / 3, i % 3) = rotation[i];}
    if (!imu_R_lidar_.allFinite() ||
      !(imu_R_lidar_.transpose() * imu_R_lidar_).isApprox(Eigen::Matrix3f::Identity(), 1e-4f) ||
      std::abs(imu_R_lidar_.determinant() - 1.f) > 1e-4f)
    {throw std::runtime_error("Invalid LiDAR extrinsic rotation");}
    const auto translation = declare_parameter<std::vector<double>>(
      "lidar_translation_in_imu", {0., 0., 0.});
    if (translation.size() != 3) {throw std::runtime_error("Expected 3 extrinsic translation values");}
    for (int i = 0; i < 3; ++i) {imu_T_lidar_(i) = translation[i];}
    if (!imu_T_lidar_.allFinite()) {throw std::runtime_error("Invalid LiDAR extrinsic translation");}
    const auto base_transform = declare_parameter<std::vector<double>>(
      "base_transform_in_imu", std::vector<double>{});
    if (base_transform.size() != 16) {
      throw std::runtime_error("base_transform_in_imu must be supplied by lio.launch.py from cart.yaml");
    }
    for (int i = 0; i < 16; ++i) {imu_T_base_(i / 4, i % 4) = base_transform[i];}
    const Eigen::Matrix3f base_rotation = imu_T_base_.block<3, 3>(0, 0);
    if (!imu_T_base_.allFinite() ||
      !imu_T_base_.row(3).isApprox(Eigen::RowVector4f(0, 0, 0, 1), 1e-5f) ||
      !(base_rotation.transpose() * base_rotation).isApprox(Eigen::Matrix3f::Identity(), 1e-4f) ||
      std::abs(base_rotation.determinant() - 1.f) > 1e-4f)
    {throw std::runtime_error("Invalid base_link mounting transform");}
    leaf_ = declare_parameter<double>("source_voxel_size", 0.5);
    radius_ = declare_parameter<double>("local_map_radius", 30.0);
    period_ = declare_parameter<double>("registration_period", 0.5);
    max_age_ = declare_parameter<double>("max_scan_age", 1.0);
    pairing_ = declare_parameter<double>("odom_pairing_tolerance", 0.03);
    timeout_ = declare_parameter<double>("tracking_timeout", 2.0);
    max_failures_ = declare_parameter<int>("max_failures", 5);
    o_.resolution = declare_parameter<double>("ndt_resolution", 1.0);
    o_.epsilon = declare_parameter<double>("transformation_epsilon", 0.01);
    o_.rotation_epsilon_degree =
      declare_parameter<double>("transformation_rotation_epsilon_degree", 0.5);
    o_.max_iterations = declare_parameter<int>("max_iterations", 30);
    o_.min_points = declare_parameter<int>("min_points", 30);
    o_.max_fitness = declare_parameter<double>("max_fitness", 0.5);
    o_.min_overlap = declare_parameter<double>("min_overlap", 0.5);
    o_.max_translation = declare_parameter<double>("max_correction_translation", 2.0);
    const double max_rotation_degree =
      declare_parameter<double>("max_correction_rotation_degree", 28.6478897565);
    o_.max_rotation = max_rotation_degree * kDegreeToRadian;
    initial_o_ = o_;
    initial_o_.max_fitness = declare_parameter<double>("initial_max_fitness", 1.5);
    initial_o_.min_overlap = declare_parameter<double>("initial_min_overlap", 0.3);
    initial_o_.max_translation = declare_parameter<double>(
      "initial_max_correction_translation", 3.0);
    const double initial_max_rotation_degree =
      declare_parameter<double>("initial_max_correction_rotation_degree", 45.8366236105);
    initial_o_.max_rotation = initial_max_rotation_degree * kDegreeToRadian;
    initial_search_xy_ = declare_parameter<double>("initial_search_xy", 1.0);
    const double initial_search_yaw_degree =
      declare_parameter<double>("initial_search_yaw_degree", 15.0);
    initial_search_yaw_ = initial_search_yaw_degree * kDegreeToRadian;

    initial_refinement_translation_tolerance_ =
      declare_parameter<double>("initial_refinement_translation_tolerance", 0.1);
    const double initial_refinement_rotation_tolerance_degree = declare_parameter<double>(
      "initial_refinement_rotation_tolerance_degree", 2.0);
    initial_refinement_rotation_tolerance_ =
      initial_refinement_rotation_tolerance_degree * kDegreeToRadian;
    initial_relaxation_failures_per_step_ =
      declare_parameter<int>("initial_relaxation_failures_per_step", 3);
    initial_relaxation_max_steps_ = declare_parameter<int>("initial_relaxation_max_steps", 6);
    initial_min_overlap_limit_ = declare_parameter<double>("initial_min_overlap_limit", 0.1);
    initial_max_fitness_limit_ = declare_parameter<double>("initial_max_fitness_limit", 3.0);
    initial_max_iterations_limit_ = declare_parameter<int>("initial_max_iterations_limit", 60);
    initial_search_xy_limit_ = declare_parameter<double>("initial_search_xy_limit", 2.0);
    const double initial_search_yaw_limit_degree =
      declare_parameter<double>("initial_search_yaw_limit_degree", 30.0);
    initial_search_yaw_limit_ = initial_search_yaw_limit_degree * kDegreeToRadian;
    initial_max_translation_limit_ =
      declare_parameter<double>("initial_max_correction_translation_limit", 4.0);
    const double initial_max_rotation_limit_degree = declare_parameter<double>(
      "initial_max_correction_rotation_limit_degree", 60.0);
    initial_max_rotation_limit_ = initial_max_rotation_limit_degree * kDegreeToRadian;
    relocalization_relaxation_delay_ =
      declare_parameter<double>("relocalization_relaxation_delay", 10.0);
    relocalization_relaxation_step_duration_ =
      declare_parameter<double>("relocalization_relaxation_step_duration", 5.0);
    relocalization_relaxation_max_steps_ =
      declare_parameter<int>("relocalization_relaxation_max_steps", 6);
    relocalization_registration_period_ =
      declare_parameter<double>("relocalization_registration_period", 1.0);
    relocalization_min_overlap_limit_ =
      declare_parameter<double>("relocalization_min_overlap_limit", 0.1);
    relocalization_max_fitness_limit_ =
      declare_parameter<double>("relocalization_max_fitness_limit", 3.0);
    relocalization_max_iterations_limit_ =
      declare_parameter<int>("relocalization_max_iterations_limit", 60);
    relocalization_search_xy_limit_ =
      declare_parameter<double>("relocalization_search_xy_limit", 6.0);
    const double relocalization_search_yaw_limit_degree = declare_parameter<double>(
      "relocalization_search_yaw_limit_degree", 45.0);
    relocalization_search_yaw_limit_ =
      relocalization_search_yaw_limit_degree * kDegreeToRadian;
    relocalization_local_map_radius_limit_ =
      declare_parameter<double>("relocalization_local_map_radius_limit", 60.0);
    relocalization_max_translation_limit_ = declare_parameter<double>(
      "relocalization_max_correction_translation_limit", 6.0);
    const double relocalization_max_rotation_limit_degree = declare_parameter<double>(
      "relocalization_max_correction_rotation_limit_degree", 75.0);
    relocalization_max_rotation_limit_ =
      relocalization_max_rotation_limit_degree * kDegreeToRadian;

    recovery_tightening_successes_per_step_ =
      declare_parameter<int>("recovery_tightening_successes_per_step", 3);
    correction_speed_ = declare_parameter<double>("correction_max_speed", 0.5);
    correction_angular_speed_ = declare_parameter<double>(
      "correction_max_angular_speed_degree", 15.0) * kDegreeToRadian;
    confirmation_required_ = declare_parameter<int>("candidate_confirmation_count", 3);
    confirmation_distance_ = declare_parameter<double>("candidate_consistency_translation", 0.3);
    confirmation_angle_ = declare_parameter<double>(
      "candidate_consistency_rotation_degree", 5.0) * kDegreeToRadian;
    confirmation_max_gap_ = declare_parameter<double>("candidate_confirmation_max_gap", 3.0);
    worker_budget_ms_ = declare_parameter<double>("search_time_budget_ms", 150.0);
    search_candidates_per_job_ = declare_parameter<int>("search_candidates_per_job", 5);
    result_max_age_ = declare_parameter<double>("registration_result_max_age", 2.0);
    for (double v :
      {leaf_, radius_, period_, max_age_, pairing_, timeout_,
        o_.resolution, o_.epsilon, o_.rotation_epsilon_degree,
        o_.max_fitness, o_.min_overlap, o_.max_translation, o_.max_rotation,
        initial_o_.max_fitness, initial_o_.min_overlap, initial_o_.max_translation,
        initial_o_.max_rotation, initial_search_xy_, initial_search_yaw_,
        initial_refinement_translation_tolerance_,
        initial_refinement_rotation_tolerance_, initial_min_overlap_limit_,
        initial_max_fitness_limit_, initial_search_xy_limit_, initial_search_yaw_limit_,
        initial_max_translation_limit_, initial_max_rotation_limit_,
        relocalization_relaxation_delay_, relocalization_relaxation_step_duration_,
        relocalization_registration_period_, relocalization_min_overlap_limit_,
        relocalization_max_fitness_limit_, relocalization_search_xy_limit_,
        relocalization_search_yaw_limit_, relocalization_local_map_radius_limit_,
        relocalization_max_translation_limit_, relocalization_max_rotation_limit_,
        correction_speed_, correction_angular_speed_, confirmation_distance_,
        confirmation_angle_, confirmation_max_gap_, worker_budget_ms_, result_max_age_})
    {
      if (!std::isfinite(v)) {throw std::runtime_error("NDT parameters must be finite");}
    }
    if (o_.epsilon <= 0 || o_.max_translation <= 0 || o_.max_rotation <= 0 || leaf_ <= 0 ||
      o_.rotation_epsilon_degree <= 0 || o_.rotation_epsilon_degree >= 90 ||
      radius_ <= 0 || period_ <= 0 || max_age_ <= 0 || pairing_ < 0 || timeout_ <= 0 ||
      max_failures_ < 1 || o_.resolution <= 0 || o_.max_iterations < 2 || o_.min_points < 6 ||
      o_.min_overlap <= 0 || o_.min_overlap > 1 || o_.max_fitness <= 0 ||
      initial_o_.min_overlap <= 0 || initial_o_.min_overlap > 1 || initial_o_.max_fitness <= 0 ||
      initial_o_.max_translation <= 0 || initial_o_.max_rotation <= 0 ||
      initial_search_xy_ < 0 || initial_search_yaw_ < 0 ||
      initial_refinement_translation_tolerance_ < 0 ||
      initial_refinement_rotation_tolerance_ < 0 || initial_relaxation_failures_per_step_ < 1 ||
      initial_relaxation_max_steps_ < 1 || initial_min_overlap_limit_ <= 0 ||
      initial_min_overlap_limit_ > initial_o_.min_overlap ||
      initial_max_fitness_limit_ < initial_o_.max_fitness ||
      initial_max_iterations_limit_ < initial_o_.max_iterations ||
      initial_search_xy_limit_ < initial_search_xy_ ||
      initial_search_yaw_limit_ < initial_search_yaw_ ||
      initial_max_translation_limit_ < initial_o_.max_translation ||
      initial_max_rotation_limit_ < initial_o_.max_rotation ||
      relocalization_relaxation_delay_ <= 0 || relocalization_relaxation_step_duration_ <= 0 ||
      relocalization_relaxation_max_steps_ < 1 || relocalization_registration_period_ <= 0 ||
      relocalization_min_overlap_limit_ <= 0 ||
      relocalization_min_overlap_limit_ > o_.min_overlap ||
      relocalization_max_fitness_limit_ < o_.max_fitness ||
      relocalization_max_iterations_limit_ < o_.max_iterations ||
      relocalization_search_xy_limit_ < 0 || relocalization_search_yaw_limit_ < 0 ||
      relocalization_local_map_radius_limit_ < radius_ ||
      relocalization_max_translation_limit_ < o_.max_translation ||
      relocalization_max_rotation_limit_ < o_.max_rotation ||
      recovery_tightening_successes_per_step_ < 1 || confirmation_required_ < 2 ||
      correction_speed_ <= 0 || correction_angular_speed_ <= 0 ||
      confirmation_distance_ <= 0 || confirmation_angle_ <= 0 || confirmation_max_gap_ <= 0 ||
      worker_budget_ms_ <= 0 || result_max_age_ <= 0 || search_candidates_per_job_ < 1)
    {
      throw std::runtime_error("Invalid NDT parameters");
    }
    const auto file = declare_parameter<std::string>("map_path", "");
    map_.reset(new NdtCloud);
    if (file.empty() || pcl::io::loadPCDFile(file, *map_) < 0) {
      throw std::runtime_error("Cannot load PCD map: " + file);
    }
    map_->erase(
      std::remove_if(
        map_->begin(), map_->end(), [](const auto & p) {
          return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
        }), map_->end());
    if (map_->size() < static_cast<size_t>(o_.min_points)) {
      throw std::runtime_error("Map has too few finite points");
    }
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/localization/status", rclcpp::QoS(
        1).transient_local());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose", 10);
    imu_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/localization/imu_pose", 10);
    lidar_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/localization/lidar_pose", 10);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/pointcloud_map", rclcpp::QoS(
        1).transient_local());
    sensor_msgs::msg::PointCloud2 map_msg; pcl::toROSMsg(*map_, map_msg);
    map_msg.header.frame_id = map_frame_; map_msg.header.stamp = now(); map_pub_->publish(map_msg);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fast_lio/odometry", rclcpp::QoS(200),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onOdom(msg);});
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/fast_lio/cloud_body", rclcpp::QoS(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onCloud(msg);});
    initial_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        if (msg->header.frame_id != map_frame_) {status("initial_pose_wrong_frame");return;}
        try {
          const auto requested = poseMatrix(msg->pose.pose);
          invalidateWork();
          initial_ = requested; pending_initial_ = true; tracking_ = false;
          initial_guess_ready_ = false; relocalizing_ = false;
          recovery_tightening_ = false; failures_ = 0;
          initial_failures_ = 0;
          if (!odometry_.empty()) {
            initializeInitialGuess(odometry_.back());
          } else {
            status("initial_pose_received_waiting_for_odometry");
          }
        } catch (const std::exception & e) {status(e.what());}
      });
    worker_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {pollWorker();});
    status("waiting_for_initial_pose");
  }

  ~NdtLocalization() override
  {
    // 작업자가 참조하는 지도와 설정을 파괴하기 전에 현재 계산을 마친다.
    if (worker_.valid()) {worker_.wait();}
  }

private:
  void status(const std::string & s) {std_msgs::msg::String m;m.data = s;status_pub_->publish(m);}

  int relocalizationLevel(const double stamp) const
  {
    const double elapsed = std::max(0.0, stamp - relocalization_start_);
    if (elapsed < relocalization_relaxation_delay_) {return 0;}
    const double steps = std::floor((elapsed - relocalization_relaxation_delay_) /
      relocalization_relaxation_step_duration_);
    return steps >= relocalization_relaxation_max_steps_ - 1 ?
      relocalization_relaxation_max_steps_ : 1 + static_cast<int>(steps);
  }

  AdaptiveNdtSettings recoverySettings(const int level) const
  {
    AdaptiveNdtSettings settings;
    settings.options = o_;
    settings.level = std::clamp(level, 0, relocalization_relaxation_max_steps_);
    const double ratio = static_cast<double>(settings.level) /
      static_cast<double>(relocalization_relaxation_max_steps_);
    const auto relax = [ratio](const double strict_value, const double relaxed_value) {
        return strict_value + (relaxed_value - strict_value) * ratio;
      };
    settings.options.max_fitness = relax(o_.max_fitness, relocalization_max_fitness_limit_);
    settings.options.min_overlap = relax(o_.min_overlap, relocalization_min_overlap_limit_);
    settings.options.max_translation =
      relax(o_.max_translation, relocalization_max_translation_limit_);
    settings.options.max_rotation = relax(o_.max_rotation, relocalization_max_rotation_limit_);
    settings.options.max_iterations = static_cast<int>(std::lround(
        relax(o_.max_iterations, relocalization_max_iterations_limit_)));
    settings.search_xy = ratio * relocalization_search_xy_limit_;
    settings.search_yaw = ratio * relocalization_search_yaw_limit_;
    settings.local_map_radius = relax(radius_, relocalization_local_map_radius_limit_);
    return settings;
  }

  Eigen::Matrix4f imuToLidar() const
  {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = imu_R_lidar_;
    transform.block<3, 1>(0, 3) = imu_T_lidar_;
    return transform;
  }

  void publishPose(
    const Eigen::Matrix4f & map_T_imu, const std_msgs::msg::Header & header)
  {
    geometry_msgs::msg::PoseStamped out;
    out.header = header;
    out.header.frame_id = map_frame_;
    out.pose = toPose(map_T_imu);
    imu_pose_pub_->publish(out);
    out.pose = toPose((map_T_imu * imuToLidar()).eval());
    lidar_pose_pub_->publish(out);
    out.pose = toPose((map_T_imu * imu_T_base_).eval());
    pose_pub_->publish(out);
  }

  void startRelocalization(double stamp, const std::string & reason)
  {
    if (!relocalizing_) {
      relocalization_start_ = stamp;
      invalidateWork();
      target_map_to_odom_ = map_to_odom_;
    }
    tracking_ = false;
    pending_initial_ = false;
    relocalizing_ = true;
    recovery_tightening_ = false;
    recovery_tightening_level_ = 0;
    recovery_tightening_successes_ = 0;
    failures_ = 0;
    status("relocalizing;reason=" + reason);
  }

  void initializeInitialGuess(const nav_msgs::msg::Odometry::ConstSharedPtr & odometry)
  {
    const Eigen::Matrix4f odom_pose = poseMatrix(odometry->pose.pose);
    const Eigen::Matrix4f map_pose = initialImuPoseFromBase(initial_, odom_pose, imu_T_base_);
    map_to_odom_ = map_pose * odom_pose.inverse();
    target_map_to_odom_ = map_to_odom_;
    initial_guess_ready_ = true;
    publishPose(map_pose, odometry->header);
    status("initial_pose_visible_refining");
  }

  void onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (msg->header.frame_id != odom_frame_ || msg->child_frame_id != body_frame_) {
      status("odometry_frame_mismatch");return;
    }
    try {poseMatrix(msg->pose.pose);} catch (...) {status("invalid_odometry");return;}
    double t = rclcpp::Time(msg->header.stamp).seconds();
    if (!odometry_.empty() && t <= rclcpp::Time(odometry_.back()->header.stamp).seconds()) {
      invalidateWork();
      odometry_.clear(); tracking_ = false; pending_initial_ = false;
      initial_guess_ready_ = false; relocalizing_ = false;
      recovery_tightening_ = false; last_scan_ = -1;
     status("odometry_reset_reinitialize");
    }
    const double dt = odometry_.empty() ? 0.0 : std::clamp(
      t - rclcpp::Time(odometry_.back()->header.stamp).seconds(), 0.0, 0.2);
    odometry_.push_back(msg); if (odometry_.size() > 200) {odometry_.pop_front();}
    if (pending_initial_ && !initial_guess_ready_) {
      try {initializeInitialGuess(msg);} catch (const std::exception & e) {status(e.what());}
    }
    if (tracking_ && t - last_success_ > timeout_) {
      startRelocalization(t, "tracking_timeout");
    }
    if (tracking_ || relocalizing_ || initial_guess_ready_) {
      const Eigen::Matrix4f odom_pose = poseMatrix(msg->pose.pose);
      const Eigen::Matrix4f corrected = limitPoseCorrection(
        (map_to_odom_ * odom_pose).eval(), (target_map_to_odom_ * odom_pose).eval(),
        dt, correction_speed_, correction_angular_speed_);
      map_to_odom_ = corrected * odom_pose.inverse();
      geometry_msgs::msg::TransformStamped out;out.header = msg->header;
      out.header.frame_id = map_frame_;out.child_frame_id = odom_frame_;
      auto p = toPose(map_to_odom_);out.transform.translation.x = p.position.x;
      out.transform.translation.y = p.position.y;out.transform.translation.z = p.position.z;
      out.transform.rotation = p.orientation;
      tf_->sendTransform(out);
      publishPose((map_to_odom_ * poseMatrix(msg->pose.pose)).eval(), msg->header);
    }
  }
  struct Work
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    Eigen::Matrix4f odom, guess;
    NdtOptions options, fallback_options, tighter_options;
    double stamp = 0, radius = 0, search_xy = 0, search_yaw = 0;
    bool initial = false, recovery = false, tightening = false;
    int level = 0;
    std::size_t generation = 0, candidate_offset = 0;
  };
  struct Completed
  {
    Work work;
    NdtResult result;
    bool tighter_accepted = false;
    int candidates = 0;
    double milliseconds = 0;
  };

  void invalidateWork()
  {
    ++generation_;
    latest_cloud_.reset();
    confirmations_.reset();
    recovery_tightening_successes_ = 0;
    candidate_cursor_ = 0;
  }

  void onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (msg->header.frame_id != body_frame_) {status("deskewed_cloud_frame_mismatch");return;}
    const double t = rclcpp::Time(msg->header.stamp).seconds();
    if (t <= last_scan_) {status("non_increasing_cloud_stamp");return;}
    last_scan_ = t;
    // 대기열을 늘리지 않고 미처리 점군은 항상 최신 한 장으로 교체한다.
    latest_cloud_ = msg;
  }

  Work prepareWork(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
    const Eigen::Matrix4f & odom)
  {
    Work w;
    w.cloud = msg; w.odom = odom; w.stamp = rclcpp::Time(msg->header.stamp).seconds();
    w.generation = generation_; w.candidate_offset = candidate_cursor_;
    w.initial = pending_initial_; w.recovery = relocalizing_; w.tightening = recovery_tightening_;
    w.guess = target_map_to_odom_ * odom;
    // 검증 중인 후보를 다음 스캔에서도 먼저 확인하되 아직 출력 위치에는 반영하지 않는다.
    if (confirmations_.count() > 0) {w.guess = verification_seed_ * odom;}
    w.options = o_; w.fallback_options = o_; w.radius = radius_;
    if (w.initial) {
      w.level = std::min(initial_relaxation_max_steps_,
        initial_failures_ / initial_relaxation_failures_per_step_);
      const double ratio = static_cast<double>(w.level) / initial_relaxation_max_steps_;
      const auto mix = [ratio](double a, double b) {return a + ratio * (b - a);};
      w.fallback_options = initial_o_;
      w.fallback_options.max_fitness = mix(initial_o_.max_fitness, initial_max_fitness_limit_);
      w.fallback_options.min_overlap = mix(initial_o_.min_overlap, initial_min_overlap_limit_);
      w.fallback_options.max_translation = mix(initial_o_.max_translation, initial_max_translation_limit_);
      w.fallback_options.max_rotation = mix(initial_o_.max_rotation, initial_max_rotation_limit_);
      w.fallback_options.max_iterations = std::lround(
        mix(initial_o_.max_iterations, initial_max_iterations_limit_));
      w.search_xy = mix(initial_search_xy_, initial_search_xy_limit_);
      w.search_yaw = mix(initial_search_yaw_, initial_search_yaw_limit_);
    } else if (w.recovery || w.tightening) {
      w.level = w.recovery ? relocalizationLevel(w.stamp) : recovery_tightening_level_;
      const auto settings = recoverySettings(w.level);
      w.options = settings.options; w.fallback_options = settings.options;
      w.radius = settings.local_map_radius;
      w.search_xy = settings.search_xy; w.search_yaw = settings.search_yaw;
      w.tighter_options = recoverySettings(std::max(0, w.level - 1)).options;
    }
    return w;
  }

  Completed calculate(const Work & w) const
  {
    // 작업 스레드는 입력 복사본과 읽기 전용 지도·설정만 사용하며 ROS 상태를 변경하지 않는다.
    const auto start = std::chrono::steady_clock::now();
    Completed done; done.work = w;
    NdtCloud::Ptr input(new NdtCloud), source(new NdtCloud), target(new NdtCloud);
    pcl::fromROSMsg(*w.cloud, *input);
    input->erase(std::remove_if(input->begin(), input->end(), [](const auto & p) {
      return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
    }), input->end());
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setLeafSize(leaf_, leaf_, leaf_); voxel.setInputCloud(input); voxel.filter(*source);
    for (const auto & p : *map_) {
      if ((p.getVector3fMap() - w.guess.block<3, 1>(0, 3)).squaredNorm() < w.radius * w.radius) {
        target->push_back(p);
      }
    }
    auto elapsed = [&]() {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    };
    auto attempt = [&](const Eigen::Matrix4f & guess, const NdtOptions & options) {
      ++done.candidates;
      return matchNdt(source, target, guess, options);
    };
    done.result = attempt(w.guess, w.options);
    if (!done.result.accepted && w.initial && elapsed() < worker_budget_ms_) {
      done.result = attempt(w.guess, w.fallback_options);
    }
    // 가까운 후보부터 확인하고, 예산이 끝나면 다음 최신 스캔에서 나머지 후보를 이어 확인한다.
    if (!done.result.accepted && (w.initial || w.level > 0) &&
      source->size() >= static_cast<size_t>(w.options.min_points))
    {
      std::vector<Eigen::Vector3d> candidates;
      for (const double scale : {0.5, 1.0}) {
        const double x = scale * w.search_xy, yaw = scale * w.search_yaw;
        for (const double dx : {-x, 0.0, x}) {
          for (const double dy : {-x, 0.0, x}) {
            for (const double dyaw : {-yaw, 0.0, yaw}) {
              if (dx == 0 && dy == 0 && dyaw == 0) {continue;}
              candidates.emplace_back(dx, dy, dyaw);
            }
          }
        }
      }
      std::stable_sort(candidates.begin(), candidates.end(), [&](const auto & a, const auto & b) {
        const auto cost = [&](const auto & v) {
          return v.head(2).squaredNorm() / std::max(1e-9, w.search_xy * w.search_xy) +
            v.z() * v.z() / std::max(1e-9, w.search_yaw * w.search_yaw);
        };
        return cost(a) < cost(b);
      });
      for (size_t i = 0; i < candidates.size() && done.candidates < search_candidates_per_job_ &&
        elapsed() < worker_budget_ms_; ++i)
      {
        const auto & offset = candidates[(w.candidate_offset + i) % candidates.size()];
        Eigen::Matrix4f candidate = w.guess;
        candidate(0, 3) += offset.x(); candidate(1, 3) += offset.y();
        candidate.block<3, 3>(0, 0) = Eigen::AngleAxisf(offset.z(), Eigen::Vector3f::UnitZ()) *
          w.guess.block<3, 3>(0, 0);
        const auto result = attempt(candidate, w.fallback_options);
        if (result.accepted || result.fitness < done.result.fitness) {done.result = result;}
        if (done.result.accepted) {break;}
      }
    }
    // 더 엄격한 조건도 현재 예측 위치에서 통과하는 경우에만 다음 단계 복원을 허용한다.
    if (done.result.accepted && w.tightening && elapsed() < worker_budget_ms_) {
      NdtCloud::Ptr smaller_target(new NdtCloud);
      const double radius = recoverySettings(std::max(0, w.level - 1)).local_map_radius;
      for (const auto & p : *target) {
        if ((p.getVector3fMap() - w.guess.block<3, 1>(0, 3)).squaredNorm() < radius * radius) {
          smaller_target->push_back(p);
        }
      }
      const auto strict = matchNdt(source, smaller_target, w.guess, w.tighter_options);
      const Eigen::Matrix4f delta = done.result.pose.inverse() * strict.pose;
      done.tighter_accepted = strict.accepted &&
        delta.block<3, 1>(0, 3).norm() <= confirmation_distance_ &&
        std::abs(Eigen::AngleAxisf(delta.block<3, 3>(0, 0)).angle()) <= confirmation_angle_;
    }
    done.milliseconds = elapsed();
    return done;
  }

  void pollWorker()
  {
    if (worker_.valid()) {
      if (worker_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {return;}
      try {applyResult(worker_.get());}
      catch (const std::exception & e) {
        confirmations_.reset(); recovery_tightening_successes_ = 0;
        status(std::string("registration_exception;") + e.what());
      }
    }
    if (!latest_cloud_ || (!tracking_ && !pending_initial_ && !relocalizing_) ||
      (pending_initial_ && !initial_guess_ready_)) {return;}
    const double stamp = rclcpp::Time(latest_cloud_->header.stamp).seconds();
    const double age = now().seconds() - stamp;
    if (age > max_age_ || age < -0.1) {latest_cloud_.reset();return;}
    const int level = relocalizing_ ? relocalizationLevel(stamp) :
      (recovery_tightening_ ? recovery_tightening_level_ : 0);
    const double period = level > 0 ? relocalization_registration_period_ : period_;
    const auto clock = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(clock - last_work_start_).count() < period) {return;}
    auto closest = odometry_.end(); double difference = pairing_;
    for (auto it = odometry_.begin(); it != odometry_.end(); ++it) {
      const double d = std::abs(rclcpp::Time((*it)->header.stamp).seconds() - stamp);
      if (d <= difference) {difference = d; closest = it;}
    }
    // 점군보다 늦게 도착한 같은 시각의 오도메트리는 다음 타이머에서 대응시킨다.
    if (closest == odometry_.end()) {return;}
    const auto work = prepareWork(latest_cloud_, poseMatrix((*closest)->pose.pose));
    latest_cloud_.reset(); last_work_start_ = clock;
    worker_ = std::async(std::launch::async, [this, work]() {return calculate(work);});
  }

  void applyResult(const Completed & done)
  {
    const auto & w = done.work;
    if (w.generation != generation_) {return;}
    const double age = now().seconds() - w.stamp;
    if (age > result_max_age_ || age < -0.1) {
      confirmations_.reset(); recovery_tightening_successes_ = 0;
      status("registration_result_expired");return;
    }
    const auto & result = done.result;
    std::ostringstream detail;
    detail << result.reason << ";fitness=" << result.fitness << ";overlap=" << result.overlap <<
      ";iterations=" << result.iterations << ";registration_ms=" << done.milliseconds <<
      ";candidates=" << done.candidates;
    if (w.recovery || w.tightening) {detail << ";recovery_level=" << w.level;}
    if (w.initial) {detail << ";relaxation_level=" << w.level;}
    if (!result.accepted) {
      candidate_cursor_ += std::max(1, done.candidates - (w.initial ? 2 : 1));
      confirmations_.reset(); recovery_tightening_successes_ = 0;
      if (pending_initial_) {
        // 카운터는 완화 한계에서 포화시켜 무기한 재시도 시 정수 넘침을 방지한다.
        if (initial_failures_ / initial_relaxation_failures_per_step_ < initial_relaxation_max_steps_) {
          ++initial_failures_;
        }
        detail << ";initial_pose_kept;refining";
      } else if (tracking_ && ++failures_ >= max_failures_) {
        startRelocalization(w.stamp, result.reason);
        detail << ";relocalizing";
      } else if (relocalizing_) {detail << ";relocalizing_continuously";}
      status(detail.str());return;
    }
    const Eigen::Matrix4f correction = result.pose * w.odom.inverse();
    const bool needs_confirmation = w.initial || w.recovery || w.tightening;
    if (needs_confirmation) {
      const bool confirmed = confirmations_.observe(correction, w.odom, w.stamp,
        confirmation_required_, confirmation_distance_, confirmation_angle_, confirmation_max_gap_);
      verification_seed_ = correction;
      detail << ";candidate_confirmations=" << confirmations_.count();
      if (!confirmed) {
        recovery_tightening_successes_ = 0;
        detail << ";verifying_candidate";
        status(detail.str());return;
      }
    }
    candidate_cursor_ = 0;
    // NDT 완료 시점의 오래된 자세를 발행하지 않고 목표 map→odom만 갱신한다.
    // 실제 출력은 최신 오도메트리 콜백에서 시간 기반 속도 제한을 적용한다.
    target_map_to_odom_ = correction;
    const bool recovered = relocalizing_;
    if (pending_initial_) {
      const Eigen::Matrix4f displayed = map_to_odom_ * w.odom;
      const Eigen::Matrix4f delta = displayed.inverse() * result.pose;
      if (delta.block<3, 1>(0, 3).norm() <= initial_refinement_translation_tolerance_ &&
        std::abs(Eigen::AngleAxisf(delta.block<3, 3>(0, 0)).angle()) <= initial_refinement_rotation_tolerance_)
      {
        pending_initial_ = false; tracking_ = true; initial_failures_ = 0;
        confirmations_.reset(); detail << ";initial_refinement_complete";
      } else {detail << ";refining_initial_pose";}
    } else if (recovered) {
      relocalizing_ = false; tracking_ = true;
      recovery_tightening_level_ = w.level;
      recovery_tightening_ = w.level > 0;
      recovery_tightening_successes_ = 0;
      detail << ";recovered";
    } else if (recovery_tightening_) {
      if (done.tighter_accepted) {++recovery_tightening_successes_;}
      else {recovery_tightening_successes_ = 0;}
      if (recovery_tightening_successes_ >= recovery_tightening_successes_per_step_) {
        --recovery_tightening_level_; recovery_tightening_successes_ = 0;
      }
      if (recovery_tightening_level_ == 0) {
        recovery_tightening_ = false; confirmations_.reset();
        detail << ";strict_tracking_restored";
      } else {detail << ";tightening_after_recovery";}
    }
    failures_ = 0; last_success_ = w.stamp;
    status(detail.str());
  }
  // 상태 변경은 단일 ROS 스레드에서, 무거운 계산만 비동기 작업 스레드에서 수행한다.
  std::future<Completed> worker_;
  rclcpp::TimerBase::SharedPtr worker_timer_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;
  std::chrono::steady_clock::time_point last_work_start_{};
  std::size_t generation_ = 0, candidate_cursor_ = 0;
  ConsistentCorrection confirmations_;
  int confirmation_required_, search_candidates_per_job_;
  Eigen::Matrix4f target_map_to_odom_ = Eigen::Matrix4f::Identity(),
    verification_seed_ = Eigen::Matrix4f::Identity();
  NdtOptions o_, initial_o_;NdtCloud::Ptr map_;
  Eigen::Matrix3f imu_R_lidar_ = Eigen::Matrix3f::Identity();
  Eigen::Matrix4f imu_T_base_ = Eigen::Matrix4f::Identity();
  Eigen::Vector3f imu_T_lidar_ = Eigen::Vector3f::Zero();
  std::string map_frame_, odom_frame_, body_frame_;
  double leaf_, radius_, period_, max_age_, pairing_, timeout_,
    initial_search_xy_, initial_search_yaw_,
    initial_refinement_translation_tolerance_, initial_refinement_rotation_tolerance_,
    initial_min_overlap_limit_, initial_max_fitness_limit_, initial_search_xy_limit_,
    initial_search_yaw_limit_, initial_max_translation_limit_, initial_max_rotation_limit_,
    relocalization_relaxation_delay_, relocalization_relaxation_step_duration_,
    relocalization_registration_period_, relocalization_min_overlap_limit_,
    relocalization_max_fitness_limit_, relocalization_search_xy_limit_,
    relocalization_search_yaw_limit_, relocalization_local_map_radius_limit_,
    relocalization_max_translation_limit_, relocalization_max_rotation_limit_,
    correction_speed_, correction_angular_speed_, confirmation_distance_, confirmation_angle_,
    confirmation_max_gap_, worker_budget_ms_, result_max_age_,
    last_scan_ = -1,
    last_success_ = 0, relocalization_start_ = 0;
  int max_failures_, failures_ = 0, initial_failures_ = 0,
    initial_relaxation_failures_per_step_, initial_relaxation_max_steps_,
    initial_max_iterations_limit_, relocalization_relaxation_max_steps_,
    relocalization_max_iterations_limit_, recovery_tightening_successes_per_step_,
    recovery_tightening_level_ = 0, recovery_tightening_successes_ = 0;
  bool tracking_ = false, pending_initial_ = false, initial_guess_ready_ = false,
    relocalizing_ = false, recovery_tightening_ = false;
  Eigen::Matrix4f map_to_odom_ = Eigen::Matrix4f::Identity(),
    initial_ = Eigen::Matrix4f::Identity();
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odometry_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr imu_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lidar_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);try {
    rclcpp::spin(std::make_shared<velodyne_slam::NdtLocalization>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ndt_localization"), "%s", e.what());rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();return 0;
}
