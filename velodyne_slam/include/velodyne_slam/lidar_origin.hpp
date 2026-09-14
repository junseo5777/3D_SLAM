#pragma once

#include <Eigen/Geometry>
#include <cmath>

namespace velodyne_slam
{
// 설정된 평행이동값은 IMU 축으로 표현한 라이다 원점이다.
// 오도메트리는 전역 좌표계에서 IMU의 자세와 위치를 나타낸다.
inline bool lidar_origin_in_world(
  const Eigen::Vector3d & imu_position, const Eigen::Quaterniond & imu_orientation,
  const Eigen::Vector3d & lidar_translation_in_imu, Eigen::Vector3d & origin)
{
  const double norm = imu_orientation.norm();
  if (!imu_position.allFinite() || !lidar_translation_in_imu.allFinite() ||
    !imu_orientation.coeffs().allFinite() || !std::isfinite(norm) || norm < 1e-6)
  {
    return false;
  }
  origin = imu_position + imu_orientation.normalized() * lidar_translation_in_imu;
  return origin.allFinite();
}
}  // velodyne_slam 네임스페이스
