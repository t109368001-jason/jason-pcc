#pragma once

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/segmentation/JPCCSegmentation.h>
#include <jpcc/segmentation/JPCCGMMSegmentationParameter.h>
#include <jpcc/segmentation/OctreeContainerSegmentation.h>

namespace jpcc::segmentation {

class JPCCGMMSegmentation : public JPCCSegmentation {
 public:
  using Base           = JPCCSegmentation;
  using LeafContainerT = octree::OctreeContainerSegmentation;
  using OctreeT = octree::JPCCOctreePointCloud<PointXYZINormal, LeafContainerT, pcl::octree::OctreeContainerEmpty>;

 protected:
  const JPCCGMMSegmentationParameter parameter_;
  OctreeT                            octree_;
  std::vector<float>                 alternateCentroids_;

 public:
  JPCCGMMSegmentation(JPCCGMMSegmentationParameter parameter);

  [[nodiscard]] size_t getNTrain() const override;

  void appendTrainSamples(const GroupOfFrame& groupOfFrame) override;

  void build() override;

  void segmentation(const FrameConstPtr& frame, FramePtr dynamicFrame, FramePtr staticFrame) override;
};

}  // namespace jpcc::segmentation