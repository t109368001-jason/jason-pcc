#pragma once

#include <fstream>
#include <string>

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/octree/OctreeContainerOccupancyInterval.h>

#include "Analyzer.h"

namespace jpcc {

class VoxelOccupancyIntervalSTDToVoxelCount : public Analyzer {
 public:
  static constexpr octree::BufferIndex BUFFER_SIZE = 3;

  using OctreeT = octree::JPCCOctreePointCloud<
      PointNormal,
      octree::OctreeContainerOccupancyInterval,
      pcl::octree::OctreeContainerEmpty,
      octree::OctreeNBuf<BUFFER_SIZE, octree::OctreeContainerOccupancyInterval, pcl::octree::OctreeContainerEmpty>>;

 protected:
  OctreeT octree_;

 public:
  VoxelOccupancyIntervalSTDToVoxelCount(const std::string& outputDir, double resolution);

  void compute(FrameConstPtr background, FrameConstPtr dynamic, FrameConstPtr other) override;

  void finalCompute() override;
};

}  // namespace jpcc
