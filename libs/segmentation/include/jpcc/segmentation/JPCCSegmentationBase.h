#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/segmentation/JPCCSegmentationParameter.h>

namespace jpcc::segmentation {

class JPCCSegmentationBase {
 public:
  using Ptr = shared_ptr<JPCCSegmentationBase>;

 protected:
  const JPCCSegmentationParameter& parameter_;

 public:
  JPCCSegmentationBase(const JPCCSegmentationParameter& parameter);

  [[nodiscard]] size_t getNTrain() const;

  virtual void appendTrainSamples(FramePtr frame) = 0;

  virtual void build() = 0;

  virtual void segmentation(const FrameConstPtr& frame, FramePtr dynamicFrame, FramePtr staticFrame) = 0;
};

}  // namespace jpcc::segmentation