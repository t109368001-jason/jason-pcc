#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/process/JPCCNormalEstimationParameter.h>

namespace jpcc::process {

class JPCCNormalEstimation {
 public:
  using Ptr    = shared_ptr<JPCCNormalEstimation>;
  using PointT = pcl::PointNormal;

 protected:
  JPCCNormalEstimationParameter param_;

 public:
  JPCCNormalEstimation(JPCCNormalEstimationParameter param);  // NOLINT(google-explicit-constructor)

  void computeInPlace(FramePtr& frame) const;

  void computeInPlaceAll(GroupOfFrame& frames, bool parallel = false) const;
};

void PCCDiagonalize(const Eigen::Matrix3d& A, Eigen::Matrix3d& Q, Eigen::Matrix3d& D);

}  // namespace jpcc::process
