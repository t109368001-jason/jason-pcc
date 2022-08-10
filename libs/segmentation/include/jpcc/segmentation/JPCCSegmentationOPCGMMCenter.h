#pragma once

#include <jpcc/common/Common.h>

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/segmentation/JPCCSegmentation.h>
#include <jpcc/segmentation/OctreeContainerSegmentationGMMCenter.h>

namespace jpcc::segmentation {

template <typename PointT>
class JPCCSegmentationOPCGMMCenter
    : virtual public JPCCSegmentation<PointT>,
      virtual public octree::JPCCOctreePointCloud<PointT, OctreeContainerSegmentationGMMCenter<PointT>> {
 public:
  static constexpr SegmentationType TYPE              = SegmentationType::GMM;
  static constexpr StaticPointType  STATIC_POINT_TYPE = StaticPointType::CENTER;

  using Ptr  = shared_ptr<JPCCSegmentationOPCGMMCenter>;
  using Base = octree::JPCCOctreePointCloud<PointT, OctreeContainerSegmentationGMMCenter<PointT>>;

  using LeafContainer = OctreeContainerSegmentationGMMCenter<PointT>;

 protected:
  std::vector<float> alternateCentroids_;

 public:
  JPCCSegmentationOPCGMMCenter(const JPCCSegmentationParameter& parameter, int startFrameNumber);

  void appendTrainSamples(FramePtr<PointT> frame) override;

  void segmentation(const FrameConstPtr<PointT>& frame,
                    FramePtr<PointT>             dynamicFrame,
                    FramePtr<PointT>             staticFrame,
                    FramePtr<PointT>             staticFrameAdded,
                    FramePtr<PointT>             staticFrameRemoved) override;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentationOPCGMMCenter.hpp>
