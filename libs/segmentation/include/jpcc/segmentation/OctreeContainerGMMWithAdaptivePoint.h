#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeContainerGMM.h>
#include <jpcc/octree/OctreeContainerAdaptivePoint.h>

namespace jpcc::segmentation {

class OctreeContainerGMMWithAdaptivePoint : virtual public octree::OctreeContainerGMM,
                                            virtual public octree::OctreeContainerAdaptivePoint {
 public:
  OctreeContainerGMMWithAdaptivePoint();

  void reset() override;

  void addPoint(const PointXYZINormal& point) override;

  void update(double alpha);
};

}  // namespace jpcc::octree