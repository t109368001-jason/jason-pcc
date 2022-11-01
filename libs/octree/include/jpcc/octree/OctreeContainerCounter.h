#pragma once

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

class OctreeContainerCounter : virtual public pcl::octree::OctreeContainerBase {
 protected:
  size_t count_;

 public:
  OctreeContainerCounter();

  void reset() override;

  void addPointIndex(const Index& index);

  [[nodiscard]] size_t getCount() const;
};

}  // namespace jpcc::octree