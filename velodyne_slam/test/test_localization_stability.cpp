#include "velodyne_slam/localization_stability.hpp"
#include <gtest/gtest.h>

using namespace velodyne_slam;

TEST(LocalizationStability, ConfirmationCompensatesVehicleMotion)
{
  ConsistentCorrection gate;
  Eigen::Matrix4f correction = Eigen::Matrix4f::Identity();
  correction(0, 3) = 4;
  for (int i = 0; i < 3; ++i) {
    Eigen::Matrix4f odom = Eigen::Matrix4f::Identity();
    odom(0, 3) = i * 2;
    odom.block<3, 3>(0, 0) = Eigen::AngleAxisf(i * 0.2f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    const Eigen::Matrix4f map_pose = correction * odom;
    EXPECT_EQ(gate.observe((map_pose * odom.inverse()).eval(), odom,
      i + 1.0, 3, 0.3, 0.1, 3.0), i == 2);
  }
}

TEST(LocalizationStability, OutlierFailureGapAndRepeatedStampResetConfirmation)
{
  ConsistentCorrection gate;
  const Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
  auto observe = [&](const Eigen::Matrix4f & candidate, double stamp) {
    return gate.observe(candidate, identity, stamp, 3, 0.3, 0.1, 3.0);
  };
  EXPECT_FALSE(observe(identity, 1));
  EXPECT_FALSE(observe(identity, 2));
  Eigen::Matrix4f outlier = identity; outlier(0, 3) = 2;
  EXPECT_FALSE(observe(outlier, 3));
  EXPECT_EQ(gate.count(), 1);
  gate.reset();
  EXPECT_FALSE(observe(identity, 4));
  EXPECT_FALSE(observe(identity, 5));
  EXPECT_FALSE(observe(identity, 9));
  EXPECT_FALSE(observe(identity, 9));
  EXPECT_EQ(gate.count(), 1);
}

TEST(LocalizationStability, AnchoredConfirmationRejectsSlowlyDriftingCandidates)
{
  ConsistentCorrection gate;
  const Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
  for (int i = 0; i < 3; ++i) {
    Eigen::Matrix4f candidate = identity; candidate(0, 3) = i * 0.2f;
    EXPECT_FALSE(gate.observe(candidate, identity, i + 1., 3, 0.3, 0.1, 3));
  }
}

TEST(LocalizationStability, CorrectionRateDoesNotDependOnUpdateFrequency)
{
  Eigen::Matrix4f target = Eigen::Matrix4f::Identity(); target(0, 3) = 4;
  target.block<3, 3>(0, 0) = Eigen::AngleAxisf(1.f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  Eigen::Matrix4f a = Eigen::Matrix4f::Identity(), b = a;
  for (int i = 0; i < 10; ++i) {a = limitPoseCorrection(a, target, 0.1, 0.5, 0.25);}
  for (int i = 0; i < 100; ++i) {b = limitPoseCorrection(b, target, 0.01, 0.5, 0.25);}
  EXPECT_TRUE(a.isApprox(b, 1e-5));
  EXPECT_NEAR(a(0, 3), 0.5, 1e-5);
  EXPECT_NEAR(std::atan2(a(1, 0), a(0, 0)), 0.25, 1e-5);
  EXPECT_TRUE(limitPoseCorrection(a, target, 0, 0.5, 0.25).isApprox(a));
}

TEST(LocalizationStability, SmallCorrectionsReachTargetWithoutOvershoot)
{
  Eigen::Matrix4f target = Eigen::Matrix4f::Identity(); target(1, 3) = 0.01f;
  EXPECT_TRUE(limitPoseCorrection(Eigen::Matrix4f::Identity(), target,
    0.1, 0.5, 0.25).isApprox(target));
}
