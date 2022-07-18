#pragma once

#include <jpcc/common/Common.h>

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/octree/OctreeContainerGMM.h>
#include <jpcc/segmentation/OctreeContainerGMMWithAdaptivePoint.h>
#include <jpcc/segmentation/JPCCSegmentationBase.h>

namespace jpcc::segmentation {

class JPCCSegmentationOPCGMMAdaptive
    : virtual public JPCCSegmentationBase,
      virtual public octree::JPCCOctreePointCloud<PointXYZINormal, OctreeContainerGMMWithAdaptivePoint> {
 public:
  static constexpr char TYPE[]              = "gmm";
  static constexpr char STATIC_POINT_TYPE[] = "adaptive";

  using Ptr  = shared_ptr<JPCCSegmentationOPCGMMAdaptive>;
  using Base = octree::JPCCOctreePointCloud<PointXYZINormal, OctreeContainerGMMWithAdaptivePoint>;

  using LeafContainer = Base::LeafContainer;

 protected:
  std::vector<float> alternateCentroids_;

 public:
  JPCCSegmentationOPCGMMAdaptive(const JPCCSegmentationParameter& parameter);

  void appendTrainSamples(FramePtr frame) override;

  void build() override;

  void segmentation(const FrameConstPtr& frame, FramePtr dynamicFrame, FramePtr staticFrame) override;
};

}  // namespace jpcc::segmentation