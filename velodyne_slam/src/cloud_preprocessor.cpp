#include "velodyne_slam/cloud_preprocessor.hpp"

#include <cmath>
#include <pcl/filters/voxel_grid.h>

namespace velodyne_slam {

Cloud::Ptr preprocess(const Cloud & input, const FilterConfig & config)
{
  Cloud::Ptr cropped(new Cloud);
  cropped->reserve(input.size());
  for (const auto & point : input) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
      continue;
    }
    const double range = std::hypot(std::hypot(point.x, point.y), point.z);
    if (range >= config.min_range && range <= config.max_range) {
      cropped->push_back(point);
    }
  }
  Cloud::Ptr output(new Cloud);
  if (cropped->empty()) {
    return output;
  }
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud(cropped);
  const auto leaf = static_cast<float>(config.voxel_size);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*output);
  return output;
}

}  // velodyne_slam 네임스페이스
