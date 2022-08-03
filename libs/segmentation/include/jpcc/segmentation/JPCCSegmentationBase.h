#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/segmentation/JPCCSegmentationParameter.h>

namespace jpcc::segmentation {

template <typename PointT>
class JPCCSegmentationBase {
 public:
  using Ptr = shared_ptr<JPCCSegmentationBase>;

 protected:
  const JPCCSegmentationParameter& parameter_;
  size_t                           startFrameNumber_;

 public:
  JPCCSegmentationBase(const JPCCSegmentationParameter& parameter);

  [[nodiscard]] size_t getNTrain() const;

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

#include <jpcc/segmentation/impl/JPCCSegmentationBase.hpp>
