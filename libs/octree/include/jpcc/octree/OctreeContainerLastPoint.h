#pragma once

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

template <typename PointT>
class OctreeContainerLastPoint : virtual public pcl::octree::OctreeContainerBase {
 protected:
  PointT lastPoint_;

 public:
  OctreeContainerLastPoint();

  void reset() override;

  virtual void addPoint(const PointT& point);

  [[nodiscard]] const PointT& getLastPoint() const;

  [[nodiscard]] PointT& getLastPoint();

  void resetLastPoint();
};

}  // namespace jpcc::octree

#include <jpcc/octree/impl/OctreeContainerLastPoint.hpp>
