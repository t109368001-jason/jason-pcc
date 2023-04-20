#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/common/IJPCCSegmentationContext.h>
#include <jpcc/segmentation/JPCCSegmentationParameter.h>

namespace jpcc::segmentation {

class JPCCSegmentation {
 public:
  using Ptr = shared_ptr<JPCCSegmentation>;

  using PointT = PointSegmentation;

 protected:
  const JPCCSegmentationParameter& parameter_;
  int                              startFrameNumber_;

 public:
  JPCCSegmentation(const JPCCSegmentationParameter& parameter, int startFrameNumber);

  virtual bool isThreadSafe();

  [[nodiscard]] virtual bool isBuilt() const = 0;

  virtual void appendTrainSamplesAndBuild(const FramePtr& frame, const PclFramePtr<PointT>& pclFrame) = 0;

  virtual void appendTrainSamplesAndBuild(const GroupOfFrame&            frames,
                                          const GroupOfPclFrame<PointT>& pclFrames,
                                          bool                           parallel);

  virtual void segmentation(IJPCCSegmentationContext& context, bool parallel);

  virtual void segmentation(const GroupOfFrame&            frames,
                            const GroupOfPclFrame<PointT>& pclFrames,
                            const GroupOfFrame&            dynamicFrames,
                            const GroupOfFrame&            staticAddedFrames,
                            const GroupOfFrame&            staticRemovedFrames) = 0;

 protected:
  virtual void segmentation(IJPCCSegmentationContext& context, size_t index) = 0;
};

}  // namespace jpcc::segmentation
