#pragma once

#include <pcl/octree/octree_container.h>

namespace jpcc::octree {

class OctreeContainerCounter : public pcl::octree::OctreeContainerBase {
 protected:
  size_t count_;

 public:
  OctreeContainerCounter();

  void reset() override;

  void addPointIndex(const pcl::index_t& index);

  [[nodiscard]] size_t getCount() const;
};

}  // namespace jpcc::octree