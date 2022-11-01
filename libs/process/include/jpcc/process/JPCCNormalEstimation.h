#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/process/JPCCNormalEstimationParameter.h>

#define PCL_NO_PRECOMPILE
#include <pcl/features/normal_3d.h>

namespace jpcc::process {

class JPCCNormalEstimation {
 public:
  using Ptr              = shared_ptr<JPCCNormalEstimation>;
  using NormalEstimation = pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal>;

 protected:
  JPCCNormalEstimationParameter param_;
  NormalEstimation              normalEstimation_;

 public:
  JPCCNormalEstimation(JPCCNormalEstimationParameter param);

  void computeInPlace(FramePtr& frame) const;

  void computeInPlaceAll(GroupOfFrame& frames, bool parallel = false) const;
};

}  // namespace jpcc::process
