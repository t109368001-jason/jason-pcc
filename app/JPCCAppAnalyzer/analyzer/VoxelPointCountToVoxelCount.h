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

  void compute(FrameConstPtr<pcl::PointXYZINormal> background,
               FrameConstPtr<pcl::PointXYZINormal> dynamic,
               FrameConstPtr<pcl::PointXYZINormal> other) override;

  void finalCompute() override;

  void getCloud(FramePtr<pcl::PointXYZINormal>& cloud) override;

  void reset() override;
};

}  // namespace jpcc
