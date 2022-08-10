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
  bool                             built_;

 public:
  JPCCSegmentation(const JPCCSegmentationParameter& parameter, int startFrameNumber);

  [[nodiscard]] virtual bool isBuilt() const;

  virtual void appendTrainSamples(FramePtr<PointT> frame) = 0;

  virtual void segmentation(const FrameConstPtr<PointT>& frame,
                            FramePtr<PointT>             dynamicFrame,
                            FramePtr<PointT>             staticFrame,
                            FramePtr<PointT>             staticFrameAdded,
                            FramePtr<PointT>             staticFrameRemoved) = 0;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentation.hpp>
