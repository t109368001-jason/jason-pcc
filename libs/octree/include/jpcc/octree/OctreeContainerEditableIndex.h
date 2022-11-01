#pragma once

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

class OctreeContainerEditableIndex : virtual public pcl::octree::OctreeContainerPointIndex {
 public:
  void setPointIndex(Index index);
};

}  // namespace jpcc::octree