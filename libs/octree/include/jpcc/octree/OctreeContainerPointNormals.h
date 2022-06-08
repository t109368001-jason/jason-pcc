#pragma once

#include <Eigen/Dense>

#include <pcl/octree/octree_container.h>

#include <jpcc/common/Common.h>

namespace jpcc::octree {

class OctreeContainerPointNormals : public pcl::octree::OctreeContainerBase {
 public:
  static constexpr int AZIMUTH_INDEX = 0;
  static constexpr int ZENITH_INDEX  = 0;

 protected:
  Eigen::MatrixX2d angles_;

 public:
  OctreeContainerPointNormals();

  void reset() override;

  void addPoint(const PointNormal& point);

  Eigen::VectorXd getAzimuths();

  Eigen::VectorXd getZeniths();
};

}  // namespace jpcc::octree