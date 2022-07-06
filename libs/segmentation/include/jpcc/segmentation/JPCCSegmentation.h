#pragma once

#include <jpcc/common/Common.h>

namespace jpcc::segmentation {

class JPCCSegmentation {
 public:
  using Ptr = shared_ptr<JPCCSegmentation>;

  virtual size_t getNTrain() const = 0;

  virtual void appendTrainSamples(const GroupOfFrame& groupOfFrame) = 0;

  virtual void build() = 0;

  virtual void segmentation(const FrameConstPtr& frame, FramePtr dynamic, FramePtr background) = 0;
};

}  // namespace jpcc::segmentation