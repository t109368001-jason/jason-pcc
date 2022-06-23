#pragma once

#include <fstream>
#include <string>

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/octree/OctreeContainerReflectivitys.h>

#include "Analyzer.h"

namespace jpcc {

class VoxelReflectivitySTDToVoxelCount : public Analyzer {
 public:
  static constexpr octree::BufferIndex BUFFER_SIZE = 3;

  using OctreeT = octree::JPCCOctreePointCloud<
      PointNormal,
      octree::OctreeContainerReflectivitys,
      pcl::octree::OctreeContainerEmpty,
      octree::OctreeNBuf<BUFFER_SIZE, octree::OctreeContainerReflectivitys, pcl::octree::OctreeContainerEmpty>>;

 protected:
  OctreeT octree_;

 public:
  VoxelReflectivitySTDToVoxelCount(const float& frequency, const double& resolution, const std::string& outputDir);

  void compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) override;

  void finalCompute() override;
};

}  // namespace jpcc
