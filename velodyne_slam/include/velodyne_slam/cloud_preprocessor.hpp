#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace velodyne_slam {

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

struct FilterConfig {
  double min_range;
  double max_range;
  double voxel_size;
};

// 기하 정보만 출력한다. 이후 ring/time 기반 왜곡 보정을 위해 원본 메시지는 별도로 유지한다.
Cloud::Ptr preprocess(const Cloud & input, const FilterConfig & config);

}  // velodyne_slam 네임스페이스
