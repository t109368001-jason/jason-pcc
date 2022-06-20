#pragma once

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

class OctreeContainerPointNormals : public pcl::octree::OctreeContainerBase {
 protected:
  std::vector<double> azimuths_;
  std::vector<double> zeniths_;

 public:
  OctreeContainerPointNormals();

  void reset() override;

  void addPoint(const PointNormal& point);

  const std::vector<double>& getAzimuths() const;

  const std::vector<double>& getZeniths() const;
};

}  // namespace jpcc::octree