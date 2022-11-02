#pragma once

#include <jpcc/segmentation/JPCCSegmentation.h>

namespace jpcc::segmentation {

class JPCCSegmentationNone : virtual public JPCCSegmentation {
 public:
  JPCCSegmentationNone(const JPCCSegmentationParameter& parameter, int startFrameNumber);

  bool isThreadSafe() override;

  bool isBuilt() const override;

  void appendTrainSamplesAndBuild(const FramePtr& frame, const PclFramePtr<PointSegmentation>& pclFrame) override;

 protected:
  void segmentation(IJPCCSegmentationContext& context, size_t index) override;
};

}  // namespace jpcc::segmentation
