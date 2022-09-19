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

  virtual void appendTrainSamplesAndBuild(FramePtr<PointT> frame) = 0;

  virtual void segmentation(const FrameConstPtr<PointT>& frame,
                            FramePtr<PointT>             dynamicFrame,
                            FramePtr<PointT>             staticFrame,
                            FramePtr<PointT>             staticAddedFrame,
                            FramePtr<PointT>             staticRemovedFrame) = 0;

  virtual void segmentation(const GroupOfFrame<PointT>& frames,
                            const GroupOfFrame<PointT>& dynamicFrames,
                            const GroupOfFrame<PointT>& staticFrames,
                            const GroupOfFrame<PointT>& staticAddedFrames,
                            const GroupOfFrame<PointT>& staticRemovedFrames,
                            bool                        parallel);
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentation.hpp>
