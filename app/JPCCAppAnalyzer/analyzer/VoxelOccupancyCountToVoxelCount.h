#pragma once

#include <fstream>
#include <string>

#include "VoxelPointCountToVoxelCount.h"

namespace jpcc {

class VoxelOccupancyCountToVoxelCount : public VoxelPointCountToVoxelCount {
 public:
  VoxelOccupancyCountToVoxelCount(const float& frequency, const double& resolution, const std::string& outputDir);

  void compute(FrameConstPtr<pcl::PointXYZINormal> background,
               FrameConstPtr<pcl::PointXYZINormal> dynamic,
               FrameConstPtr<pcl::PointXYZINormal> other) override;

  void finalCompute() override;

  void getCloud(FramePtr<pcl::PointXYZINormal>& cloud) override;

  void reset() override;
};

}  // namespace jpcc
