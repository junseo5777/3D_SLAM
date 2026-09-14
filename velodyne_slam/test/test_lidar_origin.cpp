#include <gtest/gtest.h>
#include <limits>
#include "velodyne_slam/lidar_origin.hpp"

using velodyne_slam::lidar_origin_in_world;

TEST(LidarOrigin, IdentityPoseUsesImuCoordinates)
{
  Eigen::Vector3d origin;
  ASSERT_TRUE(lidar_origin_in_world(
    {1, 2, 3}, Eigen::Quaterniond::Identity(), {0, 0, 0.05}, origin));
  EXPECT_TRUE(origin.isApprox(Eigen::Vector3d(1, 2, 3.05), 1e-12));
}

TEST(LidarOrigin, DownwardImuAxisPlacesLidarFiveCentimetresBelow)
{
  Eigen::Vector3d origin;
  // Y축으로 180도 회전하면 위쪽이 +Z인 전역 좌표계에서 IMU +Z가 아래를 향한다.
  ASSERT_TRUE(lidar_origin_in_world(
    {1, 2, 0.87}, Eigen::Quaterniond(0, 0, 1, 0), {0, 0, 0.05}, origin));
  EXPECT_TRUE(origin.isApprox(Eigen::Vector3d(1, 2, 0.82), 1e-12));
}

TEST(LidarOrigin, TiltRotatesOffsetInsteadOfSubtractingWorldHeight)
{
  Eigen::Vector3d origin;
  const double half_sqrt = std::sqrt(0.5);
  // Y축으로 90도 회전하면 IMU +Z가 전역 +X 방향을 향한다.
  ASSERT_TRUE(lidar_origin_in_world(
    {1, 2, 3}, Eigen::Quaterniond(half_sqrt, 0, half_sqrt, 0), {0, 0, 0.05}, origin));
  EXPECT_TRUE(origin.isApprox(Eigen::Vector3d(1.05, 2, 3), 1e-12));
}

TEST(LidarOrigin, NormalizesQuaternionAndRotatesAllOffsetComponents)
{
  Eigen::Vector3d origin;
  const double root_two = std::sqrt(2.0);
  ASSERT_TRUE(lidar_origin_in_world(
    {1, 2, 3}, Eigen::Quaterniond(root_two, 0, 0, root_two), {0.1, 0.2, 0.05}, origin));
  EXPECT_TRUE(origin.isApprox(Eigen::Vector3d(0.8, 2.1, 3.05), 1e-12));
}

TEST(LidarOrigin, RejectsInvalidPoseOrTranslation)
{
  Eigen::Vector3d origin;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(lidar_origin_in_world({1, 2, 3}, Eigen::Quaterniond(0, 0, 0, 0), {0, 0, 0.05}, origin));
  EXPECT_FALSE(lidar_origin_in_world({1, 2, 3}, Eigen::Quaterniond(nan, 0, 0, 0), {0, 0, 0.05}, origin));
  EXPECT_FALSE(lidar_origin_in_world({nan, 2, 3}, Eigen::Quaterniond::Identity(), {0, 0, 0.05}, origin));
  EXPECT_FALSE(lidar_origin_in_world({1, 2, 3}, Eigen::Quaterniond::Identity(), {0, 0, nan}, origin));
}
