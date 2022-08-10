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
  size_t                           startFrameNumber_;

 public:
  JPCCSegmentation(const JPCCSegmentationParameter& parameter);

  [[nodiscard]] int getNTrain() const;

  virtual void setStartFrameNumber(size_t startFrameNumber);

  virtual void appendTrainSamples(FramePtr<PointT> frame) = 0;

  virtual void build() = 0;

  virtual void segmentation(const FrameConstPtr<PointT>& frame,
                            FramePtr<PointT>             dynamicFrame,
                            FramePtr<PointT>             staticFrame,
                            FramePtr<PointT>             staticFrameAdded,
                            FramePtr<PointT>             staticFrameRemoved) = 0;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentation.hpp>
