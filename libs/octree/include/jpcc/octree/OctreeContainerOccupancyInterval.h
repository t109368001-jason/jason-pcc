#pragma once

#include <Eigen/Dense>

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

class OctreeContainerOccupancyInterval : public pcl::octree::OctreeContainerBase {
 protected:
  int             count_;
  Eigen::VectorXi occupancyIntervals_;

 public:
  OctreeContainerOccupancyInterval();

  void reset() override;

  void addPointIndex(const index_t& index);

  void appendCount();

  [[nodiscard]] Eigen::VectorXi getOccupancyIntervals() const;

  OctreeContainerOccupancyInterval& operator++();
};

}  // namespace jpcc::octree