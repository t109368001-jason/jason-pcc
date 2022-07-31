#pragma once

#include <fstream>
#include <string>

#include <jpcc/common/Common.h>
#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/octree/OctreeContainerOccupancyInterval.h>

#include "Analyzer.h"

namespace jpcc {

class VoxelOccupancyIntervalEntropyToVoxelCount : public Analyzer {
 public:
  static constexpr octree::BufferIndex BUFFER_SIZE = 3;

  using OctreeT = octree::JPCCOctreePointCloud<
      pcl::PointXYZINormal,
      octree::OctreeContainerOccupancyInterval,
      pcl::octree::OctreeContainerEmpty,
      octree::OctreeNBuf<BUFFER_SIZE, octree::OctreeContainerOccupancyInterval, pcl::octree::OctreeContainerEmpty>>;

 protected:
  OctreeT octree_;

 public:
  VoxelOccupancyIntervalEntropyToVoxelCount(const float&       frequency,
                                            const double&      resolution,
                                            const std::string& outputDir);

  void compute(FrameConstPtr<pcl::PointXYZINormal> background,
               FrameConstPtr<pcl::PointXYZINormal> dynamic,
               FrameConstPtr<pcl::PointXYZINormal> other) override;

  void finalCompute() override;

  void getCloud(FramePtr<pcl::PointXYZINormal>& cloud) override;

  void reset() override;
};

}  // namespace jpcc
