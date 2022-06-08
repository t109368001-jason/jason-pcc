#pragma once

#include <fstream>
#include <string>

#include "VoxelPointCountToVoxelCount.h"

namespace jpcc {

class VoxelOccupancyCountToVoxelCount : public VoxelPointCountToVoxelCount {
 protected:
  double resolution_;

 public:
  VoxelOccupancyCountToVoxelCount(std::string filename, double resolution);

  void compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) override;

  void finalCompute() override;
};

}  // namespace jpcc
