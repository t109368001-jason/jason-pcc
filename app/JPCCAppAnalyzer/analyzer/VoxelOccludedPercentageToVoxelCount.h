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

  using OctreeT = octree::JPCCOctreePointCloud<PointAnalyzer,
                                               octree::OctreeContainerOccludedCount<PointAnalyzer>,
                                               pcl::octree::OctreeContainerEmpty,
                                               octree::OctreeNBuf<BUFFER_SIZE,
                                                                  octree::OctreeContainerOccludedCount<PointAnalyzer>,
                                                                  pcl::octree::OctreeContainerEmpty>>;

 protected:
  const size_t quantCount_;
  OctreeT      octree_;

 public:
  VoxelOccludedPercentageToVoxelCount(const float&       frequency,
                                      const double&      resolution,
                                      const std::string& outputDir,
                                      size_t             quantResolution);

  void compute(PclFrameConstPtr<PointAnalyzer> background,
               PclFrameConstPtr<PointAnalyzer> dynamic,
               PclFrameConstPtr<PointAnalyzer> other) override;

  void finalCompute() override;

  void getCloud(PclFramePtr<PointAnalyzer>& cloud) override;

  void reset() override;
};

}  // namespace jpcc
