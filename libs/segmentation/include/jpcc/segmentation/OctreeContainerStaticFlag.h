#pragma once

#include <vector>

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::segmentation {

class OctreeContainerStaticFlag : virtual public pcl::octree::OctreeContainerBase {
 protected:
  bool isStatic_;

 public:
  OctreeContainerStaticFlag();

  void reset() override;

  bool isStatic() const;

  void setIsStatic(bool isStatic);
};

}  // namespace jpcc::segmentation