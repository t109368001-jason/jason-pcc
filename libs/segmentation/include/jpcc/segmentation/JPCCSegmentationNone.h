#pragma once

#include <jpcc/segmentation/JPCCSegmentation.h>

namespace jpcc::segmentation {

class JPCCSegmentationNone : virtual public JPCCSegmentation {
 public:
  JPCCSegmentationNone(const JPCCSegmentationParameter& parameter, int startFrameNumber);

  [[nodiscard]] bool isThreadSafe() override;

  [[nodiscard]] bool isBuilt() const override;

  void appendTrainSamplesAndBuild(const FramePtr& frame, const PclFramePtr<PointSegmentation>& pclFrame) override;

  void segmentation(const GroupOfFrame&            frames,
                    const GroupOfPclFrame<PointT>& pclFrames,
                    const GroupOfFrame&            dynamicFrames,
                    const GroupOfFrame&            staticAddedFrames,
                    const GroupOfFrame&            staticRemovedFrames) override;

 protected:
  void segmentation(IJPCCSegmentationContext& context, size_t index) override;
};

}  // namespace jpcc::segmentation
