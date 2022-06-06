#pragma once

#include <fstream>
#include <string>

#include <jpcc/octree/OctreeCounter.h>

#include "Analyzer.h"

namespace jpcc {

class VoxelPointCountToVoxelCount : public Analyzer {
 protected:
  octree::OctreeCounter<PointNormal, 3> octreeCounter_;

 public:
  VoxelPointCountToVoxelCount(std::string filename, double resolution);

  bool exists() override;

  void compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) override;

  void finalCompute() override;
};

}  // namespace jpcc
