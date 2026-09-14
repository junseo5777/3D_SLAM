#include "velodyne_slam/ndt_matcher.hpp"
#include <Eigen/Geometry>
#include <pcl/registration/ndt.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <limits>

namespace velodyne_slam
{
NdtResult matchNdt(
  const NdtCloud::ConstPtr & source,
  const NdtCloud::ConstPtr & target,
  const Eigen::Matrix4f & guess, const NdtOptions & o)
{
  NdtResult result;
  result.fitness = std::numeric_limits<double>::infinity();
  if (!guess.allFinite() || source->size() < static_cast<size_t>(o.min_points) ||
    target->size() < static_cast<size_t>(o.min_points))
  {
    result.reason = "insufficient_points_or_invalid_guess";
    return result;
  }
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  ndt.setResolution(o.resolution);
  ndt.setStepSize(o.step_size);
  ndt.setTransformationEpsilon(o.epsilon);
  // PCL은 회전 종료 기준을 각도의 코사인으로 받는다.
  ndt.setTransformationRotationEpsilon(
    std::cos(o.rotation_epsilon_degree * std::acos(-1.0) / 180.0));
  ndt.setMaximumIterations(o.max_iterations);
  ndt.setInputSource(source);
  ndt.setInputTarget(target);
  NdtCloud aligned;
  ndt.align(aligned, guess);
  result.pose = ndt.getFinalTransformation();
  result.iterations = ndt.getFinalNumIteration();
  if (!ndt.hasConverged() || result.iterations >= o.max_iterations || !result.pose.allFinite()) {
    result.reason = "not_converged";
    return result;
  }
  result.fitness = ndt.getFitnessScore();
  pcl::KdTreeFLANN<pcl::PointXYZI> tree;
  tree.setInputCloud(target);
  size_t inliers = 0;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  for (const auto & p : aligned) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
      tree.nearestKSearch(p, 1, indices, distances) > 0 &&
      distances[0] <= o.overlap_distance * o.overlap_distance) {++inliers;}
  }
  result.overlap = static_cast<double>(inliers) / aligned.size();
  const Eigen::Matrix4f delta = guess.inverse() * result.pose;
  const double angle = Eigen::AngleAxisf(delta.block<3, 3>(0, 0)).angle();
  if (!std::isfinite(result.fitness) || result.fitness > o.max_fitness ||
    result.overlap < o.min_overlap)
  {
    result.reason = "poor_match";
  } else if (delta.block<3, 1>(
      0,
      3).norm() > o.max_translation || std::abs(angle) > o.max_rotation)
  {
    result.reason = "correction_too_large";
  } else {
    result.accepted = true;
    result.reason = "tracking";
  }
  return result;
}
}
