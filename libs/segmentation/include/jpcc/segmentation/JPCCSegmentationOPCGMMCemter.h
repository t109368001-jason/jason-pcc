#pragma once

#include <jpcc/common/Common.h>

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/segmentation/JPCCSegmentationBase.h>
#include <jpcc/segmentation/OctreeContainerSegmentationGMMCenter.h>

namespace jpcc::segmentation {

class JPCCSegmentationOPCGMMCemter
    : virtual public JPCCSegmentationBase,
      virtual public octree::JPCCOctreePointCloud<PointXYZINormal, OctreeContainerSegmentationGMMCenter> {
 public:
  static constexpr SegmentationType TYPE              = SegmentationType::GMM;
  static constexpr StaticPointType  STATIC_POINT_TYPE = StaticPointType::CENTER;

  using Ptr  = shared_ptr<JPCCSegmentationOPCGMMCemter>;
  using Base = octree::JPCCOctreePointCloud<PointXYZINormal, OctreeContainerSegmentationGMMCenter>;

  using LeafContainer = Base::LeafContainer;

 protected:
  std::vector<float> alternateCentroids_;

 public:
  JPCCSegmentationOPCGMMCemter(const JPCCSegmentationParameter& parameter);

  void appendTrainSamples(FramePtr frame) override;

  void build() override;

  void segmentation(const FrameConstPtr& frame,
                    FramePtr             dynamicFrame,
                    FramePtr             staticFrame,
                    FramePtr             staticFrameAdded,
                    FramePtr             staticFrameRemoved) override;
};

}  // namespace jpcc::segmentation