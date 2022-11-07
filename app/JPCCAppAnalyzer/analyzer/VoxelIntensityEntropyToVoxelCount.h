#pragma once

#include <fstream>
#include <string>

#include <jpcc/common/Common.h>
#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/octree/OctreeContainerIntensities.h>

#include "Analyzer.h"

namespace jpcc {

class VoxelIntensityEntropyToVoxelCount : public Analyzer {
 public:
  static constexpr octree::BufferIndex BUFFER_SIZE = 3;

  using OctreeT = octree::JPCCOctreePointCloud<PointAnalyzer,
                                               octree::OctreeContainerIntensities<PointAnalyzer>,
                                               pcl::octree::OctreeContainerEmpty,
                                               octree::OctreeNBuf<BUFFER_SIZE,
                                                                  octree::OctreeContainerIntensities<PointAnalyzer>,
                                                                  pcl::octree::OctreeContainerEmpty>>;

 protected:
  OctreeT octree_;

 public:
  VoxelIntensityEntropyToVoxelCount(const float& frequency, const double& resolution, const std::string& outputDir);

  void compute(PclFrameConstPtr<PointAnalyzer> background,
               PclFrameConstPtr<PointAnalyzer> dynamic,
               PclFrameConstPtr<PointAnalyzer> other) override;

  void finalCompute() override;

  void getCloud(PclFramePtr<PointAnalyzer>& cloud) override;

  void reset() override;
};

}  // namespace jpcc
