#pragma once

#include <jpcc/segmentation/JPCCSegmentation.h>

namespace jpcc::segmentation {

template <typename PointT>
class JPCCSegmentationNone : virtual public JPCCSegmentation<PointT> {
 public:
  JPCCSegmentationNone(const JPCCSegmentationParameter& parameter, int startFrameNumber);

  bool isThreadSafe() override;

  bool isBuilt() const override;

  void appendTrainSamplesAndBuild(const FramePtr<PointT>& frame) override;

 protected:
  void segmentation(IJPCCSegmentationContext<PointT>& context, size_t index) override;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentationNone.hpp>
