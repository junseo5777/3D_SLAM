#include "velodyne_slam/initial_pose.hpp"
#include <gtest/gtest.h>

TEST(InitialPose, InvertedImuUsesRequestedLidarHeading)
{
  Eigen::Matrix3f mounting = Eigen::Vector3f(-1, 1, -1).asDiagonal();
  Eigen::Matrix4f odom = Eigen::Matrix4f::Identity();
  odom.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(0.4f, Eigen::Vector3f::UnitZ()) * mounting;
  Eigen::Matrix4f requested = Eigen::Matrix4f::Identity();
  requested.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(1.2f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  requested.block<3, 1>(0, 3) = Eigen::Vector3f(2, 3, 0.1f);
  const auto pose = velodyne_slam::initialImuPose(requested, odom, mounting);
  EXPECT_TRUE((pose.block<3, 3>(0, 0) * mounting).isApprox(
    requested.block<3, 3>(0, 0), 1e-5f));
  EXPECT_TRUE((pose.block<3, 1>(0, 3).isApprox(requested.block<3, 1>(0, 3))));
}

TEST(InitialPose, PreservesMeasuredTilt)
{
  Eigen::Matrix4f odom = Eigen::Matrix4f::Identity();
  odom.block<3, 3>(0, 0) = Eigen::AngleAxisf(0.2f, Eigen::Vector3f::UnitX()).toRotationMatrix();
  Eigen::Matrix4f requested = Eigen::Matrix4f::Identity();
  requested.block<3, 3>(0, 0) = Eigen::AngleAxisf(0.7f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  const auto pose = velodyne_slam::initialImuPose(requested, odom, Eigen::Matrix3f::Identity());
  EXPECT_TRUE((pose.block<1, 3>(2, 0).isApprox(odom.block<1, 3>(2, 0), 1e-5f)));
  EXPECT_NEAR(std::atan2(pose(1, 0), pose(0, 0)), 0.7f, 1e-5f);
}

TEST(InitialPose, BlendsTranslationAndRotationGradually)
{
  Eigen::Matrix4f current = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f target = Eigen::Matrix4f::Identity();
  target(0, 3) = 4.0f;
  target.block<3, 3>(0, 0) =
    Eigen::AngleAxisf(static_cast<float>(M_PI_2), Eigen::Vector3f::UnitZ()).toRotationMatrix();
  const Eigen::Matrix4f result = velodyne_slam::blendPose(current, target, 0.25f);
  EXPECT_NEAR(result(0, 3), 1.0f, 1e-5f);
  EXPECT_NEAR(std::atan2(result(1, 0), result(0, 0)), M_PI / 8.0, 1e-5);
}

TEST(InitialPose, RearAxleClickMatchesBaseWithInvertedImuAndTurning)
{
  Eigen::Matrix4f imu_to_base = Eigen::Matrix4f::Identity();
  imu_to_base.block<3, 3>(0, 0) = Eigen::Vector3f(-1, 1, -1).asDiagonal();
  imu_to_base.block<3, 1>(0, 3) = Eigen::Vector3f(0.70f, 0, 0.80f);
  Eigen::Matrix4f odom = Eigen::Matrix4f::Identity();
  odom.block<3, 3>(0, 0) = imu_to_base.block<3, 3>(0, 0);
  for (float yaw : {0.f, 1.5707963f, -1.f}) {
    Eigen::Matrix4f click = Eigen::Matrix4f::Identity();
    click.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    click.block<3, 1>(0, 3) = Eigen::Vector3f(2, 3, 0.1f);
    const auto imu = velodyne_slam::initialImuPoseFromBase(click, odom, imu_to_base);
    EXPECT_TRUE((imu * imu_to_base).isApprox(click, 1e-5f));
    Eigen::Matrix4f imu_to_lidar = Eigen::Matrix4f::Identity();
    imu_to_lidar.block<3, 3>(0, 0) = imu_to_base.block<3, 3>(0, 0);
    imu_to_lidar(2, 3) = 0.05f;
    const Eigen::Matrix4f base_to_lidar = (imu * imu_to_base).inverse() * imu * imu_to_lidar;
    EXPECT_TRUE((base_to_lidar.block<3, 1>(0, 3).isApprox(Eigen::Vector3f(0.7f, 0, 0.75f), 1e-5f)));
  }
}

TEST(InitialPose, RearAxleClickPreservesTiltAndPosition)
{
  Eigen::Matrix4f mount = Eigen::Matrix4f::Identity();
  mount.block<3, 1>(0, 3) = Eigen::Vector3f(-0.7f, 0.1f, -0.8f);
  Eigen::Matrix4f odom = Eigen::Matrix4f::Identity();
  odom.block<3, 3>(0, 0) = Eigen::AngleAxisf(0.3f, Eigen::Vector3f::UnitY()).toRotationMatrix();
  Eigen::Matrix4f click = Eigen::Matrix4f::Identity(); click(0, 3) = 3;
  const auto imu = velodyne_slam::initialImuPoseFromBase(click, odom, mount);
  const Eigen::Matrix4f base = imu * mount;
  EXPECT_TRUE((base.block<3, 1>(0, 3).isApprox(click.block<3, 1>(0, 3), 1e-5f)));
  EXPECT_TRUE((imu.block<1, 3>(2, 0).isApprox(odom.block<1, 3>(2, 0), 1e-5f)));
}
