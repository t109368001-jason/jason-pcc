#pragma once

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/octree/OctreeContainerGMMWithAdaptivePoint.h>
#include <jpcc/segmentation/JPCCSegmentation.h>
#include <jpcc/segmentation/JPCCGMMSegmentationParameter.h>

namespace jpcc::segmentation {

class JPCCGMMSegmentation : public JPCCSegmentation {
 public:
  using Base           = JPCCSegmentation;
  using LeafContainerT = octree::OctreeContainerGMMWithAdaptivePoint;
  using OctreeT = octree::JPCCOctreePointCloud<PointXYZINormal, LeafContainerT, pcl::octree::OctreeContainerEmpty>;

 protected:
  const JPCCGMMSegmentationParameter parameter_;
  OctreeT                            octree_;

 public:
  JPCCGMMSegmentation(JPCCGMMSegmentationParameter parameter);

  [[nodiscard]] size_t getNTrain() const override;

  void appendTrainSamples(const GroupOfFrame& groupOfFrame) override;

  void build() override;

  void segmentation(const FrameConstPtr& frame, FramePtr dynamicFrame, FramePtr staticFrame) override;
};

}  // namespace jpcc::segmentation