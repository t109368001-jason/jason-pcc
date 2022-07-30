#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeContainerAdaptivePoint.h>
#include <jpcc/segmentation/OctreeContainerGMM.h>
#include <jpcc/segmentation/OctreeContainerStaticFlag.h>

namespace jpcc::segmentation {

template <typename PointT>
class OctreeContainerSegmentationGMMCenter : virtual public OctreeContainerStaticFlag,
                                             virtual public OctreeContainerGMM<PointT> {
 public:
  OctreeContainerSegmentationGMMCenter();

  void reset() override;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/OctreeContainerSegmentationGMMCenter.hpp>
