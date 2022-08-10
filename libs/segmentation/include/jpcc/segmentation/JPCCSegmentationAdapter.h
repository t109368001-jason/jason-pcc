#pragma once

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/segmentation/JPCCSegmentation.h>
#include <jpcc/segmentation/JPCCSegmentationParameter.h>
#include <jpcc/segmentation/OctreeContainerSegmentationGMMCenter.h>

namespace jpcc::segmentation {

template <typename PointT>
class JPCCSegmentationAdapter : public JPCCSegmentation<PointT> {
 public:
  using Base = JPCCSegmentationAdapter;

 protected:
  typename JPCCSegmentation<PointT>::Ptr backend_;

 public:
  JPCCSegmentationAdapter(const JPCCSegmentationParameter& parameter, int startFrameNumber);

  [[nodiscard]] bool isBuilt() const override;

  void appendTrainSamples(FramePtr<PointT> frame) override;

  void segmentation(const FrameConstPtr<PointT>& frame,
                    FramePtr<PointT>             dynamicFrame,
                    FramePtr<PointT>             staticFrame,
                    FramePtr<PointT>             staticFrameAdded,
                    FramePtr<PointT>             staticFrameRemoved) override;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentationAdapter.hpp>
