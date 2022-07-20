#pragma once

#include <jpcc/common/Common.h>

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/segmentation/JPCCSegmentationBase.h>
#include <jpcc/segmentation/OctreeContainerGMMWithAdaptivePoint.h>

namespace jpcc::segmentation {

class JPCCSegmentationOPCGMMAdaptive
    : virtual public JPCCSegmentationBase,
      virtual public octree::JPCCOctreePointCloud<PointXYZINormal, OctreeContainerGMMWithAdaptivePoint> {
 public:
  static constexpr SegmentationType TYPE              = SegmentationType::GMM;
  static constexpr StaticPointType  STATIC_POINT_TYPE = StaticPointType::ADAPTIVE;

  using Ptr  = shared_ptr<JPCCSegmentationOPCGMMAdaptive>;
  using Base = octree::JPCCOctreePointCloud<PointXYZINormal, OctreeContainerGMMWithAdaptivePoint>;

  using LeafContainer = Base::LeafContainer;

 protected:
  bool               isFirstFrame;
  std::vector<float> alternateCentroids_;

 public:
  JPCCSegmentationOPCGMMAdaptive(const JPCCSegmentationParameter& parameter);

  void appendTrainSamples(FramePtr frame) override;

  void build() override;

  void segmentation(const FrameConstPtr& frame,
                    FramePtr             dynamicFrame,
                    FramePtr             staticFrame,
                    FramePtr             staticFrameAdded,
                    FramePtr             staticFrameRemoved) override;
};

}  // namespace jpcc::segmentation