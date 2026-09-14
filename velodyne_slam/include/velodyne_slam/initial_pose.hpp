#pragma once
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace velodyne_slam
{
// 지정 위치를 유지한 채 기준 프레임의 진행 방향에 맞는 IMU 자세를 계산한다.
// 측정한 IMU 기울기는 유지하고 전역 yaw만 바꿔 중력 정렬을 보존한다.
inline Eigen::Matrix4f initialImuPose(
  const Eigen::Matrix4f & requested, const Eigen::Matrix4f & odometry,
  const Eigen::Matrix3f & imu_R_lidar)
{
  const Eigen::Vector3f forward =
    odometry.block<3, 3>(0, 0) * imu_R_lidar.col(0);
  if (!forward.allFinite() || forward.head<2>().norm() < 1e-6f) {
    throw std::runtime_error("initial_pose_heading_undefined");
  }
  const float desired_yaw = std::atan2(requested(1, 0), requested(0, 0));
  const float current_yaw = std::atan2(forward.y(), forward.x());
  Eigen::Matrix4f result = requested;
  result.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(desired_yaw - current_yaw, Eigen::Vector3f::UnitZ()) *
    odometry.block<3, 3>(0, 0);
  return result;
}

// 뒷바퀴 중심의 클릭 위치와 카트 방향을 내부 IMU 위치로 변환한다.
// 기울기는 실측값을 유지하고, 회전된 장착 오프셋까지 고려해 클릭 위치를 정확히 맞춘다.
inline Eigen::Matrix4f initialImuPoseFromBase(
  const Eigen::Matrix4f & requested, const Eigen::Matrix4f & odometry,
  const Eigen::Matrix4f & imu_T_base)
{
  Eigen::Matrix4f result = initialImuPose(
    requested, odometry, imu_T_base.block<3, 3>(0, 0));
  result.block<3, 1>(0, 3) = requested.block<3, 1>(0, 3) -
    result.block<3, 3>(0, 0) * imu_T_base.block<3, 1>(0, 3);
  return result;
}

// 현재 자세에서 목표 자세까지 gain 비율만 이동해 초기 위치가 갑자기 튀지 않게 한다.
inline Eigen::Matrix4f blendPose(
  const Eigen::Matrix4f & current, const Eigen::Matrix4f & target, const float gain)
{
  if (!current.allFinite() || !target.allFinite() || !std::isfinite(gain) ||
    gain <= 0.0f || gain > 1.0f)
  {
    throw std::runtime_error("invalid_pose_blend_input");
  }
  Eigen::Quaternionf current_rotation(current.block<3, 3>(0, 0));
  Eigen::Quaternionf target_rotation(target.block<3, 3>(0, 0));
  if (current_rotation.norm() < 1e-6f || target_rotation.norm() < 1e-6f) {
    throw std::runtime_error("invalid_pose_blend_rotation");
  }
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  result.block<3, 1>(0, 3) =
    (1.0f - gain) * current.block<3, 1>(0, 3) + gain * target.block<3, 1>(0, 3);
  const Eigen::Quaternionf blended_rotation =
    current_rotation.normalized().slerp(gain, target_rotation.normalized()).normalized();
  result.block<3, 3>(0, 0) = blended_rotation.toRotationMatrix();
  return result;
}
}
