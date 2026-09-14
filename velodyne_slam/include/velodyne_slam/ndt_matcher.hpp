#pragma once
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace velodyne_slam
{
using NdtCloud = pcl::PointCloud<pcl::PointXYZI>;
struct NdtOptions
{
  double resolution = 1.0, step_size = 0.1, epsilon = 0.01;
  double rotation_epsilon_degree = 0.5;  // 반복 간 회전 변화 종료 기준(도)
  int max_iterations = 30, min_points = 30;
  double max_fitness = 0.5, overlap_distance = 1.0, min_overlap = 0.5;
  double max_translation = 2.0, max_rotation = 0.5;
};
struct NdtResult
{
  bool accepted = false;
  std::string reason;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double fitness = 0.0, overlap = 0.0;
  int iterations = 0;
};
NdtResult matchNdt(
  const NdtCloud::ConstPtr & source,
  const NdtCloud::ConstPtr & target,
  const Eigen::Matrix4f & guess, const NdtOptions & options);
}
