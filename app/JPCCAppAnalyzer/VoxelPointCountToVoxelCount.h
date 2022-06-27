#pragma once

#include <fstream>
#include <string>

#include <jpcc/octree/OctreeCounter.h>

#include "Analyzer.h"

namespace jpcc {

class VoxelPointCountToVoxelCount : public Analyzer {
 public:
  using OctreeT = octree::OctreeCounter<3>;

 protected:
  OctreeT octreeCounter_;

 protected:
  VoxelPointCountToVoxelCount(const float&       frequency,
                              const double&      resolution,
                              const std::string& outputDir,
                              const std::string& title);

 public:
  VoxelPointCountToVoxelCount(const float& frequency, const double& resolution, const std::string& outputDir);

  void compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) override;

  void finalCompute() override;
};

}  // namespace jpcc
