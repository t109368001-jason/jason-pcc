#pragma once

#include <jpcc/segmentation/JPCCSegmentation.h>

namespace jpcc::segmentation {

template <typename PointT>
class JPCCSegmentationNone : virtual public JPCCSegmentation<PointT> {
 public:
  JPCCSegmentationNone(const JPCCSegmentationParameter& parameter, int startFrameNumber);

  bool isBuilt() const override;

  void appendTrainSamplesAndBuild(const FramePtr<PointT>& frame) override;

  void segmentation(const FrameConstPtr<PointT>& frame,
                    const FramePtr<PointT>&      dynamicFrame,
                    const FramePtr<PointT>&      staticFrame,
                    const FramePtr<PointT>&      staticAddedFrame,
                    const FramePtr<PointT>&      staticRemovedFrame) override;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentationNone.hpp>
