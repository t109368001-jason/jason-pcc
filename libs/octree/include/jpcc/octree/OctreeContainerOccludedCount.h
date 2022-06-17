#pragma once

#include <array>
#include <list>

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

class OctreeContainerOccludedCount : public pcl::octree::OctreeContainerBase {
 public:
  using Count1D = std::vector<size_t>;
  using Count2D = std::vector<Count1D>;
  using Count3D = std::vector<Count2D>;

 protected:
  Count3D                count3D_;
  std::list<PointNormal> pointBuffer_;

 public:
  OctreeContainerOccludedCount();

  void reset() override;

  void addPoint(const PointNormal& point);

  void compute(const Eigen::Vector3f& min_pt, const Eigen::Vector3f& max_pt, size_t quantCount);

  float getMinimumOccludedPercentage();

  float getXYOccludedPercentage();

  float getXZOccludedPercentage();

  float getYZOccludedPercentage();

  void initCountMatrix(size_t quantCount);
};

}  // namespace jpcc::octree