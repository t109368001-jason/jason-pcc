#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeContainerLastPoint.h>

namespace jpcc::octree {

template <typename PointT>
class OctreeContainerAdaptivePoint : virtual public OctreeContainerLastPoint<PointT> {
 protected:
  PointT adaptivePoint_;

 public:
  OctreeContainerAdaptivePoint();

  void reset() override;

  virtual void addPoint(const PointT& point);

  void updateAdaptivePoint(double alpha);

  [[nodiscard]] const PointT& getAdaptivePoint() const;

  [[nodiscard]] PointT& getAdaptivePoint();
};

}  // namespace jpcc::octree

#include <jpcc/octree/impl/OctreeContainerAdaptivePoint.hpp>
