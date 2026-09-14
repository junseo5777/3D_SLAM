#include <nav_msgs/msg/odometry.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "velodyne_slam/lidar_origin.hpp"
#include "velodyne_slam/consecutive_free_space.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using MapCloud = pcl::PointCloud<pcl::PointXYZI>;
using Key = std::array<int64_t, 3>;

struct KeyHash
{
  size_t operator()(const Key & key) const
  {
    size_t hash = 0;
    std::hash<int64_t> hasher;
    for (const auto value : key) {
      hash ^= hasher(value) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    return hash;
  }
};

struct VoxelState
{
  pcl::PointXYZI point;
  velodyne_slam::ConsecutiveFreeSpace free_space_streak;
};

struct OdomSample
{
  double stamp;
  std::array<double, 3> lidar_origin;
};

class MapBuilder : public rclcpp::Node
{
public:
  MapBuilder()
  : Node("map_builder")
  {
    leaf_ = declare_parameter<double>("map_voxel_size", 0.2);
    max_voxels_ = declare_parameter<int>("max_voxels", 1000000);
    path_ = declare_parameter<std::string>("map_path", "");
    frame_ = declare_parameter<std::string>("map_frame", "map");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
    save_on_shutdown_ = declare_parameter<bool>("save_on_shutdown", true);
    dynamic_clearing_enabled_ = declare_parameter<bool>("dynamic_clearing_enabled", true);
    clearing_miss_threshold_ = declare_parameter<int>("clearing_miss_threshold", 5);
    clearing_max_range_ = declare_parameter<double>("clearing_max_range", 30.0);
    clearing_endpoint_margin_ = declare_parameter<double>("clearing_endpoint_margin", 0.4);
    clearing_ray_stride_ = declare_parameter<int>("clearing_ray_stride", 4);
    odom_pairing_tolerance_ = declare_parameter<double>("odom_pairing_tolerance", 0.03);
    const auto translation = declare_parameter<std::vector<double>>(
      "lidar_translation_in_imu", std::vector<double>{});
    if (translation.size() != 3 || !std::all_of(
        translation.begin(), translation.end(), [](double value) {return std::isfinite(value);}))
    {
      throw std::runtime_error(
              "lidar_translation_in_imu must contain 3 finite values from lidar_imu_calibration.yaml");
    }
    lidar_translation_in_imu_ = Eigen::Vector3d(translation[0], translation[1], translation[2]);

    if (!std::isfinite(leaf_) || leaf_ <= 0 || max_voxels_ <= 0 || path_.empty() ||
      clearing_miss_threshold_ <= 0 || !std::isfinite(clearing_max_range_) ||
      clearing_max_range_ <= 0 || !std::isfinite(clearing_endpoint_margin_) ||
      clearing_endpoint_margin_ < leaf_ || clearing_ray_stride_ <= 0 ||
      !std::isfinite(odom_pairing_tolerance_) || odom_pairing_tolerance_ < 0)
    {
      throw std::runtime_error("Invalid map builder parameters");
    }

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/pointcloud_map", rclcpp::QoS(1).transient_local());
    status_ = create_publisher<std_msgs::msg::String>(
      "/mapping/status", rclcpp::QoS(1).transient_local());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fast_lio/odometry", rclcpp::QoS(50),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {handle_odom(*msg);});
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/fast_lio/cloud_world", rclcpp::QoS(2),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {handle_cloud(*msg);});

    timer_ = create_wall_timer(
      std::chrono::seconds(2), [this] {
        if (!dirty_ || voxels_.empty()) {return;}
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*snapshot(), msg);
        msg.header.frame_id = frame_;
        msg.header.stamp = now();
        pub_->publish(msg);
        dirty_ = false;
      });

    service_ = create_service<std_srvs::srv::Trigger>(
      "/save_map",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = save_map(response->message);
        report(response->message);
      });
    report("waiting_for_fast_lio");
  }

  ~MapBuilder() override
  {
    if (!save_on_shutdown_ || saved_) {return;}
    std::string message;
    if (save_map(message)) {
      RCLCPP_INFO(get_logger(), "Automatic shutdown save: %s", message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Automatic shutdown save skipped: %s", message.c_str());
    }
  }

private:
  bool make_key(double x, double y, double z, Key & key) const
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      std::abs(x / leaf_) > 1e12 || std::abs(y / leaf_) > 1e12 ||
      std::abs(z / leaf_) > 1e12)
    {
      return false;
    }
    key = {
      static_cast<int64_t>(std::floor(x / leaf_)),
      static_cast<int64_t>(std::floor(y / leaf_)),
      static_cast<int64_t>(std::floor(z / leaf_))};
    return true;
  }

  void handle_odom(const nav_msgs::msg::Odometry & msg)
  {
    if (msg.header.frame_id != frame_ || msg.child_frame_id != imu_frame_) {return;}
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    Eigen::Vector3d origin;
    if (!velodyne_slam::lidar_origin_in_world(
        Eigen::Vector3d(p.x, p.y, p.z), Eigen::Quaterniond(q.w, q.x, q.y, q.z),
        lidar_translation_in_imu_, origin))
    {
      return;
    }
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    if (!odom_buffer_.empty() && stamp <= odom_buffer_.back().stamp) {
      odom_buffer_.clear();
    }
    odom_buffer_.push_back({stamp, {origin.x(), origin.y(), origin.z()}});
    while (odom_buffer_.size() > 100) {odom_buffer_.pop_front();}
  }

  bool find_sensor_origin(double stamp, std::array<double, 3> & origin) const
  {
    double best_difference = std::numeric_limits<double>::infinity();
    for (auto it = odom_buffer_.rbegin(); it != odom_buffer_.rend(); ++it) {
      const double difference = std::abs(it->stamp - stamp);
      if (difference < best_difference) {
        best_difference = difference;
        origin = it->lidar_origin;
      }
      if (it->stamp < stamp - odom_pairing_tolerance_) {break;}
    }
    return best_difference <= odom_pairing_tolerance_;
  }

  size_t clear_observed_free_space(
    const MapCloud & cloud, const std::unordered_set<Key, KeyHash> & hit_keys,
    const std::array<double, 3> & origin)
  {
    std::unordered_set<Key, KeyHash> missed_keys;
    const double step = leaf_ * 0.5;
    const size_t first = scan_count_ % static_cast<size_t>(clearing_ray_stride_);

    for (size_t i = first; i < cloud.size(); i += clearing_ray_stride_) {
      const auto & point = cloud[i];
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const double dx = point.x - origin[0];
      const double dy = point.y - origin[1];
      const double dz = point.z - origin[2];
      const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
      const double free_range = std::min(range - clearing_endpoint_margin_, clearing_max_range_);
      if (!std::isfinite(range) || range <= 0 || free_range <= leaf_) {continue;}
      const double inverse_range = 1.0 / range;
      for (double distance = leaf_; distance <= free_range; distance += step) {
        Key key;
        if (!make_key(
            origin[0] + dx * inverse_range * distance,
            origin[1] + dy * inverse_range * distance,
            origin[2] + dz * inverse_range * distance, key))
        {
          continue;
        }
        if (hit_keys.find(key) == hit_keys.end() && voxels_.find(key) != voxels_.end()) {
          missed_keys.insert(key);
        }
      }
    }

    size_t cleared = 0;
    for (const auto & key : missed_keys) {
      auto voxel = voxels_.find(key);
      if (voxel == voxels_.end()) {continue;}
      if (voxel->second.free_space_streak.observe(scan_count_, clearing_miss_threshold_)) {
        voxels_.erase(voxel);
        ++cleared;
      }
    }
    return cleared;
  }

  void handle_cloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (msg.header.frame_id != frame_) {report("world_frame_mismatch");return;}
    if (reset_required_) {return;}
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    if (stamp <= last_stamp_) {
      reset_required_ = true;
      report("non_increasing_cloud_stamp_restart_mapping");
      return;
    }
    last_stamp_ = stamp;

    MapCloud cloud;
    pcl::fromROSMsg(msg, cloud);
    bool full = false;
    bool changed = false;
    std::unordered_set<Key, KeyHash> hit_keys;
    hit_keys.reserve(cloud.size());

    for (const auto & point : cloud) {
      Key key;
      if (!make_key(point.x, point.y, point.z, key)) {continue;}
      hit_keys.insert(key);
      auto existing = voxels_.find(key);
      if (existing != voxels_.end()) {
        existing->second.free_space_streak.reset();
        continue;
      }
      if (voxels_.size() >= static_cast<size_t>(max_voxels_)) {
        full = true;
        continue;
      }
      voxels_.emplace(key, VoxelState{point, {}});
      changed = true;
    }

    size_t cleared = 0;
    bool clearing_has_pose = false;
    if (dynamic_clearing_enabled_) {
      std::array<double, 3> origin;
      clearing_has_pose = find_sensor_origin(stamp, origin);
      if (clearing_has_pose) {
        cleared = clear_observed_free_space(cloud, hit_keys, origin);
      }
    }
    ++scan_count_;

    if (changed || cleared > 0) {saved_ = false;}
    dirty_ = true;
    std::string state = full ? "map_capacity_reached_save_and_stop" : "mapping";
    state += ";voxels=" + std::to_string(voxels_.size());
    state += ";cleared=" + std::to_string(cleared);
    if (dynamic_clearing_enabled_ && !clearing_has_pose) {state += ";clearing=waiting_for_odometry";}
    report(state);
  }

  bool save_map(std::string & message)
  {
    if (voxels_.empty()) {
      message = "No map points received";
      return false;
    }
    try {
      std::filesystem::path output(path_);
      if (output.has_parent_path()) {std::filesystem::create_directories(output.parent_path());}
      const auto temporary = path_ + ".tmp";
      if (pcl::io::savePCDFileBinary(temporary, *snapshot()) < 0) {
        throw std::runtime_error("PCD write failed");
      }
      std::filesystem::rename(temporary, output);
      saved_ = true;
      message = "Saved " + std::to_string(voxels_.size()) + " points to " + path_;
      return true;
    } catch (const std::exception & e) {
      message = e.what();
      return false;
    }
  }

  MapCloud::Ptr snapshot() const
  {
    MapCloud::Ptr output(new MapCloud);
    output->reserve(voxels_.size());
    for (const auto & entry : voxels_) {output->push_back(entry.second.point);}
    return output;
  }

  void report(const std::string & state)
  {
    std_msgs::msg::String message;
    message.data = state;
    status_->publish(message);
  }

  double leaf_ = 0.2;
  double last_stamp_ = -1;
  double clearing_max_range_ = 30.0;
  double clearing_endpoint_margin_ = 0.4;
  double odom_pairing_tolerance_ = 0.03;
  int max_voxels_ = 1000000;
  int clearing_miss_threshold_ = 5;
  int clearing_ray_stride_ = 4;
  std::string path_;
  std::string frame_;
  std::string imu_frame_;
  Eigen::Vector3d lidar_translation_in_imu_ = Eigen::Vector3d::Zero();
  bool dirty_ = false;
  bool reset_required_ = false;
  bool save_on_shutdown_ = true;
  bool saved_ = false;
  bool dynamic_clearing_enabled_ = true;
  size_t scan_count_ = 0;
  std::unordered_map<Key, VoxelState, KeyHash> voxels_;
  std::deque<OdomSample> odom_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MapBuilder>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("map_builder"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
