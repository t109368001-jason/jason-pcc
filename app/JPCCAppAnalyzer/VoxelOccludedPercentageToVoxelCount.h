#pragma once

#include <fstream>
#include <string>

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/octree/OctreeContainerOccludedCount.h>
#include <jpcc/octree/OctreeNBuf.h>

#include "Analyzer.h"

namespace jpcc {

class VoxelOccludedPercentageToVoxelCount : public Analyzer {
 public:
  static constexpr octree::BufferIndex BUFFER_SIZE = 3;

  using OctreeT = octree::JPCCOctreePointCloud<
      PointNormal,
      octree::OctreeContainerOccludedCount,
      pcl::octree::OctreeContainerEmpty,
      octree::OctreeNBuf<BUFFER_SIZE, octree::OctreeContainerOccludedCount, pcl::octree::OctreeContainerEmpty>>;

 protected:
  const size_t quantResolution_;
  const size_t quantCount_;
  OctreeT      octree_;

 public:
  VoxelOccludedPercentageToVoxelCount(const float&       frequency,
                                      const double&      resolution,
                                      const std::string& outputDir,
                                      size_t             quantResolution);

  void compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) override;

  void finalCompute() override;
};

}  // namespace jpcc