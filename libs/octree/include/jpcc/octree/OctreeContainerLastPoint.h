#pragma once

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

class OctreeContainerLastPoint : virtual public pcl::octree::OctreeContainerBase {
 protected:
  PointXYZINormal lastPoint_;

 public:
  OctreeContainerLastPoint();

  void reset() override;

  virtual void addPoint(const PointXYZINormal& point);

  [[nodiscard]] const PointXYZINormal& getLastPoint() const;

  [[nodiscard]] PointXYZINormal& getLastPoint();

  void resetLastPoint();
};

}  // namespace jpcc::octree