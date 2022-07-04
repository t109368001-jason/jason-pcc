#pragma once

#include <fstream>
#include <string>

#include "VoxelPointCountToVoxelCount.h"

namespace jpcc {

class VoxelOccupancyCountToVoxelCount : public VoxelPointCountToVoxelCount {
 public:
  VoxelOccupancyCountToVoxelCount(const float& frequency, const double& resolution, const std::string& outputDir);

  void compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) override;

  void finalCompute() override;

  void getCloud(FramePtr cloud) override;

  void reset() override;
};

}  // namespace jpcc
