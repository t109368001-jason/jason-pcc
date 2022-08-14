#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeContainerAdaptivePoint.h>
#include <jpcc/segmentation/OctreeContainerGMM.h>
#include <jpcc/segmentation/OctreeContainerStaticFlag.h>

namespace jpcc::segmentation {

template <typename GMMContainerT, typename StaticPointContainerT>
class OctreeContainerSegmentation : virtual public GMMContainerT, virtual public StaticPointContainerT {
 public:
  OctreeContainerSegmentation();

  void reset() override;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/OctreeContainerSegmentation.hpp>
