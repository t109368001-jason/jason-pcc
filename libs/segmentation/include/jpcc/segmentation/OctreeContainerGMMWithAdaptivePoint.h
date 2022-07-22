#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeContainerAdaptivePoint.h>
#include <jpcc/segmentation/OctreeContainerGMM.h>
#include <jpcc/segmentation/OctreeContainerStaticFlag.h>

namespace jpcc::segmentation {

class OctreeContainerGMMWithAdaptivePoint : virtual public OctreeContainerStaticFlag,
                                            virtual public OctreeContainerGMM,
                                            virtual public octree::OctreeContainerAdaptivePoint {
 public:
  OctreeContainerGMMWithAdaptivePoint();

  void reset() override;

  void addPoint(const PointXYZINormal& point) override;

  void appendTrainSamples(double alpha);

  void updateModel(double alpha, double minimumVariance) override;
};

}  // namespace jpcc::segmentation