#pragma once

#include <vector>

#include <boost/dynamic_bitset.hpp>

#include <Eigen/Dense>

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

class OctreeContainerOccludedCount : virtual public pcl::octree::OctreeContainerBase {
 public:
  using Count3D = boost::dynamic_bitset<>;

 protected:
  Count3D                      count3D_;
  std::vector<PointXYZINormal> pointBuffer_;

 public:
  OctreeContainerOccludedCount();

  void reset() override;

  void addPoint(const PointXYZINormal& point);

  void compute(const Eigen::Vector3f& min_pt, const Eigen::Vector3f& max_pt, size_t quantCount);

  [[nodiscard]] float getMinimumOccludedPercentage(size_t quantCount);

  [[nodiscard]] float getXYOccludedPercentage(size_t quantCount);

  [[nodiscard]] float getXZOccludedPercentage(size_t quantCount);

  [[nodiscard]] float getYZOccludedPercentage(size_t quantCount);
};

}  // namespace jpcc::octree