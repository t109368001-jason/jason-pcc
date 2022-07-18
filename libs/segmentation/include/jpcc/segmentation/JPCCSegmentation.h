#pragma once

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/segmentation/JPCCSegmentationBase.h>
#include <jpcc/segmentation/JPCCSegmentationParameter.h>
#include <jpcc/segmentation/OctreeContainerGMMWithAdaptivePoint.h>

namespace jpcc::segmentation {

class JPCCSegmentation : public JPCCSegmentationBase {
 public:
  using Base = JPCCSegmentation;

 protected:
  JPCCSegmentationBase::Ptr octree_;

 public:
  JPCCSegmentation(const JPCCSegmentationParameter& parameter);

  void appendTrainSamples(const GroupOfFrame& groupOfFrame);

  void appendTrainSamples(FramePtr frame) override;

  void build() override;

  void segmentation(const FrameConstPtr& frame,
                    FramePtr             dynamicFrame,
                    FramePtr             staticFrame,
                    FramePtr             staticFrameAdded,
                    FramePtr             staticFrameRemoved) override;
};

}  // namespace jpcc::segmentation