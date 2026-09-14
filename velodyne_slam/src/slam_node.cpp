#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>

#include "velodyne_slam/cloud_preprocessor.hpp"

namespace velodyne_slam {

class SlamNode : public rclcpp::Node {
public:
  SlamNode() : Node("velodyne_slam")
  {
    mode_ = declare_parameter<std::string>("mode", "mapping");
    const auto topic = declare_parameter<std::string>("input_topic", "/velodyne_points");
    const auto reliability = declare_parameter<std::string>("input_reliability", "best_effort");
    auto input_qos = rclcpp::SensorDataQoS();
    if (reliability == "reliable") {
      input_qos.reliable();
    } else if (reliability != "best_effort") {
      throw std::invalid_argument("input_reliability must be reliable or best_effort");
    }
    // 이후 정합과 지도 입출력 기능을 연결하기 위해 미리 선언한 파라미터다.
    declare_parameter<std::string>("map_path", "");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    config_ = {
      declare_parameter<double>("min_range", 1.0),
      declare_parameter<double>("max_range", 80.0),
      declare_parameter<double>("voxel_size", 0.3)};
    if (mode_ != "mapping" && mode_ != "localization") {
      throw std::invalid_argument("mode must be mapping or localization");
    }
    if (!std::isfinite(config_.min_range) || !std::isfinite(config_.max_range) ||
        !std::isfinite(config_.voxel_size) || config_.min_range < 0.0 ||
        config_.max_range <= config_.min_range || config_.voxel_size <= 0.0) {
      throw std::invalid_argument("invalid range or voxel_size parameters");
    }
    filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/filtered_points", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "~/status", rclcpp::QoS(1).reliable().transient_local());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, input_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onCloud(msg); });
    status("waiting_for_cloud");
    RCLCPP_WARN(get_logger(),
      "Scaffold mode: %s. Preprocessing only; registration, map I/O and pose/TF output are not implemented.",
      mode_.c_str());
  }

private:
  void status(const std::string & state)
  {
    std_msgs::msg::String message;
    message.data = "mode=" + mode_ + ";state=" + state + ";registration=not_implemented";
    status_pub_->publish(message);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message)
  {
    // 이후 점별 시간과 왜곡 보정을 추가할 때는 이 원본 메시지를 보존해야 한다.
    if (message->header.frame_id.empty() || message->height == 0 ||
        message->width == 0 || message->is_bigendian ||
        static_cast<uint64_t>(message->row_step) <
          static_cast<uint64_t>(message->width) * message->point_step ||
        message->data.size() != static_cast<uint64_t>(message->row_step) * message->height) {
      status("invalid_cloud_layout");
      return;
    }
    for (const std::string name : {"x", "y", "z", "intensity"}) {
      const auto field = std::find_if(message->fields.begin(), message->fields.end(),
        [&name](const auto & value) { return value.name == name; });
      if (field == message->fields.end() || field->count != 1 ||
          field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
          static_cast<uint64_t>(field->offset) + sizeof(float) > message->point_step) {
        status("invalid_xyzi_fields");
        return;
      }
    }
    Cloud input;
    pcl::fromROSMsg(*message, input);
    const auto filtered = preprocess(input, config_);
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    output.header = message->header;
    filtered_pub_->publish(output);
    status(filtered->empty() ? "empty_filtered_cloud" : "preprocessing_only");
    // 확장 지점: 정합 후 모드에 따라 지도 갱신 또는 위치추정을 연결한다.
  }

  std::string mode_;
  FilterConfig config_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

}  // velodyne_slam 네임스페이스

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<velodyne_slam::SlamNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("velodyne_slam"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
