#pragma once

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

class OctreeContainerOccupancyInterval : virtual public pcl::octree::OctreeContainerBase {
 protected:
  int              count_;
  std::vector<int> occupancyIntervals_;

 public:
  OctreeContainerOccupancyInterval();

  void reset() override;

  void addPointIndex(const Index& index);

  void appendCount();

  [[nodiscard]] const std::vector<int>& getOccupancyIntervals() const;

  OctreeContainerOccupancyInterval& operator++();
};

}  // namespace jpcc::octree