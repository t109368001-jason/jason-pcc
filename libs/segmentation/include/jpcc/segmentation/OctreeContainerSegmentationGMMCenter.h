#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeContainerAdaptivePoint.h>
#include <jpcc/segmentation/OctreeContainerGMM.h>
#include <jpcc/segmentation/OctreeContainerStaticFlag.h>

namespace jpcc::segmentation {

class OctreeContainerSegmentationGMMCenter : virtual public OctreeContainerStaticFlag,
                                             virtual public OctreeContainerGMM {
 public:
  OctreeContainerSegmentationGMMCenter();

  void reset() override;
};

}  // namespace jpcc::segmentation