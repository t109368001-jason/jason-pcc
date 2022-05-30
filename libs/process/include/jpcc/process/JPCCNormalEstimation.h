#pragma once

#include <pcl/features/normal_3d.h>

#include <jpcc/common/Common.h>
#include <jpcc/process/JPCCNormalEstimationParameter.h>

namespace jpcc::process {

template <typename PointT = Point>
class JPCCNormalEstimation {
 public:
  using Ptr              = shared_ptr<JPCCNormalEstimation>;
  using Frame            = jpcc::Frame<PointT>;
  using FramePtr         = typename Frame::Ptr;
  using GroupOfFrame     = jpcc::GroupOfFrame<PointT>;
  using NormalEstimation = pcl::NormalEstimation<PointT, PointNormal>;

 protected:
  JPCCNormalEstimationParameter param_;
  NormalEstimation              normalEstimation_;

 public:
  JPCCNormalEstimation(JPCCNormalEstimationParameter param);

  void computeInPlace(FramePtr& frame) const;

  [[nodiscard]] jpcc::Frame<PointNormal>::Ptr compute(FramePtr& frame) const;

  void computeInPlaceAll(GroupOfFrame& frames, bool parallel = false) const;

  [[nodiscard]] jpcc::GroupOfFrame<PointNormal> computeAll(GroupOfFrame& frames, bool parallel = false) const;
};

}  // namespace jpcc::process

#include <jpcc/process/impl/JPCCNormalEstimation.hpp>