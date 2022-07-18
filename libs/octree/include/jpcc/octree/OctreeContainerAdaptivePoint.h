#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeContainerLastPoint.h>

namespace jpcc::octree {

class OctreeContainerAdaptivePoint : virtual public OctreeContainerLastPoint {
 protected:
  PointXYZINormal adaptivePoint_;

 public:
  OctreeContainerAdaptivePoint();

  void reset() override;

  virtual void addPoint(const PointXYZINormal& point);

  void updateAdaptivePoint(double alpha);

  [[nodiscard]] const PointXYZINormal& getAdaptivePoint() const;

  [[nodiscard]] PointXYZINormal& getAdaptivePoint();
};

}  // namespace jpcc::octree