#pragma once

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

class OctreeContainerReflectivitys : public pcl::octree::OctreeContainerBase {
 protected:
  std::vector<Reflectivity> reflectivitys_;

 public:
  OctreeContainerReflectivitys();

  void reset() override;

  void addPoint(const PointNormal& point);

  const std::vector<Reflectivity>& getReflectivitys() const;
};

}  // namespace jpcc::octree