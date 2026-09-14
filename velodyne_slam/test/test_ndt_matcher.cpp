#include "velodyne_slam/ndt_matcher.hpp"
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <random>

using namespace velodyne_slam;
static NdtCloud::Ptr scene()
{
  NdtCloud::Ptr cloud(new NdtCloud);
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> u(-4.0, 4.0);
  std::normal_distribution<float> noise(0.0, 0.02);
  for (int i = 0; i < 9000; ++i) {
    pcl::PointXYZI p; p.x = u(rng);p.y = u(rng);p.z = u(rng);p.intensity = 1;
    if (i % 3 == 0) {p.x = 3.0 + noise(rng);}
    if (i % 3 == 1) {p.y = -2.0 + noise(rng);}
    if (i % 3 == 2) {p.z = -1.0 + noise(rng);}
    cloud->push_back(p);
  }
  return cloud;
}
TEST(NdtMatcher, RecoversKnownPoseFromNearbyPrediction)
{
  auto map = scene();NdtCloud::Ptr scan(new NdtCloud);
  Eigen::Matrix4f truth = Eigen::Matrix4f::Identity();
  truth.block<3, 3>(0, 0) = Eigen::AngleAxisf(0.04, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  truth.block<3, 1>(0, 3) = Eigen::Vector3f(0.2, -0.1, 0.1);
  pcl::transformPointCloud(*map, *scan, truth.inverse());
  NdtOptions options;options.max_iterations = 60;options.epsilon = 0.001;options.resolution = 0.5;
  const auto result = matchNdt(scan, map, Eigen::Matrix4f::Identity(), options);
  ASSERT_TRUE(result.accepted) << result.reason << " fitness=" << result.fitness;
  EXPECT_LT((result.pose.block<3, 1>(0, 3) - truth.block<3, 1>(0, 3)).norm(), 0.1);
  EXPECT_GT(result.overlap, 0.9);
}
TEST(NdtMatcher, RejectsOutOfMapPrediction)
{
  auto map = scene();Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();guess(0, 3) = 100;
  EXPECT_FALSE(matchNdt(map, map, guess, NdtOptions{}).accepted);
}
TEST(NdtMatcher, RejectsEmptyScan)
{
  NdtCloud::Ptr empty(new NdtCloud);
  EXPECT_FALSE(matchNdt(empty, scene(), Eigen::Matrix4f::Identity(), NdtOptions{}).accepted);
}
