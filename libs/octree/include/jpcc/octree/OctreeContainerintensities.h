#pragma once

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

class OctreeContainerIntensities : public pcl::octree::OctreeContainerBase {
 protected:
  std::vector<float> intensities_;

 public:
  OctreeContainerIntensities();

  void reset() override;

  void addPoint(const PointXYZINormal& point);

  const std::vector<float>& getIntensities() const;
};

}  // namespace jpcc::octree