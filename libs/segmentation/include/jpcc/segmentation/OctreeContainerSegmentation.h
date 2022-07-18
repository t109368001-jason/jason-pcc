#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeContainerGMM.h>
#include <jpcc/octree/OctreeContainerAdaptivePoint.h>

namespace jpcc::octree {

class OctreeContainerSegmentation : virtual public OctreeContainerGMM, virtual public OctreeContainerAdaptivePoint {
 public:
  OctreeContainerSegmentation();

  void reset() override;

  void addPoint(const PointXYZINormal& point) override;
};

}  // namespace jpcc::octree