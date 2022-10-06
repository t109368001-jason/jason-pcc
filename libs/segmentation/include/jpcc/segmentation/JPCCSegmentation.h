#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/segmentation/JPCCSegmentationParameter.h>

namespace jpcc::segmentation {

template <typename PointT>
class JPCCSegmentation {
 public:
  using Ptr = shared_ptr<JPCCSegmentation>;

 protected:
  const JPCCSegmentationParameter& parameter_;
  int                              startFrameNumber_;

 public:
  JPCCSegmentation(const JPCCSegmentationParameter& parameter, int startFrameNumber);

  virtual bool isThreadSafe();

  [[nodiscard]] virtual bool isBuilt() const = 0;

  virtual void appendTrainSamplesAndBuild(const FramePtr<PointT>& frame) = 0;

  virtual void appendTrainSamplesAndBuild(const GroupOfFrame<PointT>& frames, bool parallel);

  virtual void segmentation(IJPCCSegmentationContext<PointT>& context, bool parallel);

 protected:
  virtual void segmentation(IJPCCSegmentationContext<PointT>& context, size_t index) = 0;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentation.hpp>
