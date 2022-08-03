#pragma once

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/segmentation/JPCCSegmentationBase.h>
#include <jpcc/segmentation/JPCCSegmentationParameter.h>
#include <jpcc/segmentation/OctreeContainerSegmentationGMMCenter.h>

namespace jpcc::segmentation {

template <typename PointT>
class JPCCSegmentation : public JPCCSegmentationBase<PointT> {
 public:
  using Base = JPCCSegmentation;

 protected:
  typename JPCCSegmentationBase<PointT>::Ptr backend_;

 public:
  JPCCSegmentation(const JPCCSegmentationParameter& parameter);

  void appendTrainSamples(const GroupOfFrame<PointT>& groupOfFrame);

  void setStartFrameNumber(size_t startFrameNumber) override;

  void appendTrainSamples(FramePtr<PointT> frame) override;

  void build() override;

  void segmentation(const FrameConstPtr<PointT>& frame,
                    FramePtr<PointT>             dynamicFrame,
                    FramePtr<PointT>             staticFrame,
                    FramePtr<PointT>             staticFrameAdded,
                    FramePtr<PointT>             staticFrameRemoved) override;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentation.hpp>
