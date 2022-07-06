#pragma once

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

class OctreeContainerAdaptivePoint : virtual public pcl::octree::OctreeContainerBase {
 protected:
  PointXYZINormal              point_;
  std::vector<PointXYZINormal> points_;

 public:
  OctreeContainerAdaptivePoint();

  void reset() override;

  virtual void addPoint(const PointXYZINormal& point);

  void updatePoint(double alpha);

  [[nodiscard]] const PointXYZINormal& getPoint() const;

  [[nodiscard]] PointXYZINormal& getPoint();
};

}  // namespace jpcc::octree