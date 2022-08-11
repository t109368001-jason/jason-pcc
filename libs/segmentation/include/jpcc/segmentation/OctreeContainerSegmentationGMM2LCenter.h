#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeContainerAdaptivePoint.h>
#include <jpcc/segmentation/OctreeContainerGMM2L.h>
#include <jpcc/segmentation/OctreeContainerStaticFlag.h>

namespace jpcc::segmentation {

template <typename PointT>
class OctreeContainerSegmentationGMM2LCenter : virtual public OctreeContainerStaticFlag,
                                               virtual public OctreeContainerGMM2L<PointT> {
 public:
  OctreeContainerSegmentationGMM2LCenter();

  void reset() override;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/OctreeContainerSegmentationGMM2LCenter.hpp>
