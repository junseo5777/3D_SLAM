#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace velodyne_slam
{
// 이동체의 실제 이동은 그대로 두고 지도 보정으로 생기는 위치·회전 변화만 제한한다.
inline Eigen::Matrix4f limitPoseCorrection(
  const Eigen::Matrix4f & current, const Eigen::Matrix4f & target,
  double dt, double speed, double angular_speed)
{
  Eigen::Matrix4f out = current;
  const Eigen::Vector3f delta = target.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);
  const double distance = delta.norm();
  const double translation_gain = distance > 1e-9 ?
    std::min(1.0, speed * std::max(0.0, dt) / distance) : 1.0;
  out.block<3, 1>(0, 3) += static_cast<float>(translation_gain) * delta;
  const Eigen::Quaternionf a(current.block<3, 3>(0, 0)), b(target.block<3, 3>(0, 0));
  const double angle = a.angularDistance(b);
  const double rotation_gain = angle > 1e-9 ?
    std::min(1.0, angular_speed * std::max(0.0, dt) / angle) : 1.0;
  out.block<3, 3>(0, 0) = a.slerp(rotation_gain, b).normalized().toRotationMatrix();
  return out;
}

// map→odom 후보를 비교하므로 이동 중에도 같은 지도 보정을 지지하는지 확인할 수 있다.
class ConsistentCorrection
{
public:
  void reset() {count_ = 0;}
  int count() const {return count_;}
  bool observe(
    const Eigen::Matrix4f & candidate, const Eigen::Matrix4f & odometry,
    double stamp, int required, double distance_limit, double angle_limit, double max_gap)
  {
    const Eigen::Matrix4f a = anchor_ * odometry, b = candidate * odometry;
    const bool consistent = count_ > 0 && stamp > last_stamp_ &&
      stamp - last_stamp_ <= max_gap &&
      (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm() <= distance_limit &&
      Eigen::Quaternionf(a.block<3, 3>(0, 0)).angularDistance(
        Eigen::Quaternionf(b.block<3, 3>(0, 0))) <= angle_limit;
    if (!consistent) {anchor_ = candidate; count_ = 0;}
    count_ = std::min(required, count_ + 1);
    last_stamp_ = stamp;
    return count_ >= required;
  }
private:
  Eigen::Matrix4f anchor_ = Eigen::Matrix4f::Identity();
  int count_ = 0;
  double last_stamp_ = 0;
};
}  // velodyne_slam 네임스페이스
