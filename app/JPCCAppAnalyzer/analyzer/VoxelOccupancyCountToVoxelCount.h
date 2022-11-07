#pragma once

#include <fstream>
#include <string>

#include "VoxelPointCountToVoxelCount.h"

namespace jpcc {

class VoxelOccupancyCountToVoxelCount : public VoxelPointCountToVoxelCount {
 public:
  VoxelOccupancyCountToVoxelCount(const float& frequency, const double& resolution, const std::string& outputDir);

  void compute(PclFrameConstPtr<PointAnalyzer> background,
               PclFrameConstPtr<PointAnalyzer> dynamic,
               PclFrameConstPtr<PointAnalyzer> other) override;

  void finalCompute() override;

  void getCloud(PclFramePtr<PointAnalyzer>& cloud) override;

  void reset() override;
};

}  // namespace jpcc
