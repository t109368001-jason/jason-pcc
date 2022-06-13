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
  VoxelPointCountToVoxelCount(const std::string& outputDir, const std::string& filename, double resolution);

  VoxelPointCountToVoxelCount(const std::string& outputDir, double resolution);

  void compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) override;

  void finalCompute() override;
};

}  // namespace jpcc
