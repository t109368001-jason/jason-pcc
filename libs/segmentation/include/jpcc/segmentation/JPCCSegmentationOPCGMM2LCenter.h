#pragma once

#include <array>

#include <jpcc/common/Common.h>

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/segmentation/JPCCSegmentation.h>
#include <jpcc/segmentation/OctreeContainerSegmentationGMM2LCenter.h>

namespace jpcc::segmentation {

template <typename PointT>
class JPCCSegmentationOPCGMM2LCenter
    : virtual public JPCCSegmentation<PointT>,
      virtual public octree::JPCCOctreePointCloud<PointT, OctreeContainerSegmentationGMM2LCenter<PointT>> {
 public:
  static constexpr int              SIZE              = OctreeContainerSegmentationGMM2LCenter<PointT>::SIZE;
  static constexpr SegmentationType TYPE              = SegmentationType::GMM_2L;
  static constexpr StaticPointType  STATIC_POINT_TYPE = StaticPointType::CENTER;

  using Ptr  = shared_ptr<JPCCSegmentationOPCGMM2LCenter>;
  using Base = octree::JPCCOctreePointCloud<PointT, OctreeContainerSegmentationGMM2LCenter<PointT>>;

  using LeafContainer = OctreeContainerSegmentationGMM2LCenter<PointT>;

 protected:
  std::vector<float>     alternateCentroids_;
  std::array<bool, SIZE> builtVector;

 public:
  JPCCSegmentationOPCGMM2LCenter(const JPCCSegmentationParameter& parameter, int startFrameNumber);

  [[nodiscard]] bool isBuilt() const override;

  void appendTrainSamples(FramePtr<PointT> frame) override;

  void segmentation(const FrameConstPtr<PointT>& frame,
                    FramePtr<PointT>             dynamicFrame,
                    FramePtr<PointT>             staticFrame,
                    FramePtr<PointT>             staticFrameAdded,
                    FramePtr<PointT>             staticFrameRemoved) override;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentationOPCGMM2LCenter.hpp>
