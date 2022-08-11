#pragma once

#include <vector>

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::segmentation {

class OctreeContainerStaticFlag : virtual public pcl::octree::OctreeContainerBase {
 protected:
  bool isLastStatic_;

 public:
  OctreeContainerStaticFlag();

  void reset() override;

  bool isLastStatic() const;

  void setIsLastStatic(bool isStatic);
};

}  // namespace jpcc::segmentation