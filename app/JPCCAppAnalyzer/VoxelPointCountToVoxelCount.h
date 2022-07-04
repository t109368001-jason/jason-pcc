#pragma once

#include <fstream>
#include <string>

#include <jpcc/octree/OctreeCounter.h>

#include "Analyzer.h"

namespace jpcc {

class VoxelPointCountToVoxelCount : public Analyzer {
 public:
  static constexpr octree::BufferIndex BUFFER_SIZE = 3;

  using OctreeT = octree::OctreeCounter<BUFFER_SIZE>;

 protected:
  OctreeT octree_;

 protected:
  VoxelPointCountToVoxelCount(const float&       frequency,
                              const double&      resolution,
                              const std::string& outputDir,
                              const std::string& title);

 public:
  VoxelPointCountToVoxelCount(const float& frequency, const double& resolution, const std::string& outputDir);

  void compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) override;

  void finalCompute() override;

  void getCloud(FramePtr cloud) override;

  void reset() override;
};

}  // namespace jpcc
