#pragma once

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

class OctreeContainerOccupancyChangeCount : virtual public pcl::octree::OctreeContainerBase {
 protected:
  bool   previous_;
  bool   current_;
  size_t count_;

 public:
  OctreeContainerOccupancyChangeCount();

  void reset() override;

  void addPointIndex(const Index& index);

  void resetCurrent();

  void compute();

  [[nodiscard]] size_t getCount() const;
};

}  // namespace jpcc::octree