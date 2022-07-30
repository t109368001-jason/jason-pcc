#pragma once

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

template <typename PointT>
class OctreeContainerIntensities : virtual public pcl::octree::OctreeContainerBase {
 protected:
  std::vector<float> intensities_;

 public:
  OctreeContainerIntensities();

  void reset() override;

  void addPoint(const PointT& point);

  [[nodiscard]] const std::vector<float>& getIntensities() const;
};

}  // namespace jpcc::octree

#include <jpcc/octree/impl/OctreeContainerIntensities.hpp>
