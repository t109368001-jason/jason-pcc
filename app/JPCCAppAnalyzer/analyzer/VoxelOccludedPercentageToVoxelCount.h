#pragma once

#include <fstream>
#include <string>

#include <jpcc/common/Common.h>
#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/octree/OctreeContainerOccludedCount.h>
#include <jpcc/octree/OctreeNBuf.h>

#include "Analyzer.h"

namespace jpcc {

class VoxelOccludedPercentageToVoxelCount : public Analyzer {
 public:
  static constexpr octree::BufferIndex BUFFER_SIZE = 3;

  using OctreeT =
      octree::JPCCOctreePointCloud<pcl::PointXYZINormal,
                                   octree::OctreeContainerOccludedCount<pcl::PointXYZINormal>,
                                   pcl::octree::OctreeContainerEmpty,
                                   octree::OctreeNBuf<BUFFER_SIZE,
                                                      octree::OctreeContainerOccludedCount<pcl::PointXYZINormal>,
                                                      pcl::octree::OctreeContainerEmpty>>;

 protected:
  const size_t quantCount_;
  OctreeT      octree_;

 public:
  VoxelOccludedPercentageToVoxelCount(const float&       frequency,
                                      const double&      resolution,
                                      const std::string& outputDir,
                                      size_t             quantResolution);

  void compute(FrameConstPtr<pcl::PointXYZINormal> background,
               FrameConstPtr<pcl::PointXYZINormal> dynamic,
               FrameConstPtr<pcl::PointXYZINormal> other) override;

  void finalCompute() override;

  void getCloud(FramePtr<pcl::PointXYZINormal>& cloud) override;

  void reset() override;
};

}  // namespace jpcc
