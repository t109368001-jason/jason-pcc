#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/process/JPCCNormalEstimationParameter.h>

#define PCL_NO_PRECOMPILE
#include <pcl/features/normal_3d.h>

namespace jpcc::process {

template <typename PointIn, typename PointOut>
class JPCCNormalEstimation {
 public:
  using Ptr              = shared_ptr<JPCCNormalEstimation>;
  using NormalEstimation = pcl::NormalEstimation<PointIn, PointOut>;

 protected:
  JPCCNormalEstimationParameter param_;
  NormalEstimation              normalEstimation_;

 public:
  JPCCNormalEstimation(JPCCNormalEstimationParameter param);

  void computeInPlace(FramePtr<PointIn>& frame) const;

  [[nodiscard]] FramePtr<PointOut> compute(FramePtr<PointIn>& frame) const;

  void computeInPlaceAll(GroupOfFrame<PointIn>& frames, bool parallel = false) const;

  [[nodiscard]] GroupOfFrame<PointOut> computeAll(GroupOfFrame<PointIn>& frames, bool parallel = false) const;
};

}  // namespace jpcc::process

#include <jpcc/process/impl/JPCCNormalEstimation.hpp>
