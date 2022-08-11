#pragma once

#include <array>

#include <jpcc/common/Common.h>

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/segmentation/JPCCSegmentation.h>

namespace jpcc::segmentation {

template <typename PointT, typename LeafContainerT>
class JPCCSegmentationOPCGMMCenter : virtual public JPCCSegmentation<PointT>,
                                     virtual public octree::JPCCOctreePointCloud<PointT, LeafContainerT> {
 public:
  static constexpr int SIZE = LeafContainerT::SIZE;

  using Ptr  = shared_ptr<JPCCSegmentationOPCGMMCenter>;
  using Base = octree::JPCCOctreePointCloud<PointT, LeafContainerT>;

 protected:
  std::vector<float>     alternateCentroids_;
  std::array<bool, SIZE> builtVector;

 public:
  JPCCSegmentationOPCGMMCenter(const JPCCSegmentationParameter& parameter, int startFrameNumber);

  [[nodiscard]] bool isBuilt() const override;

  void appendTrainSamples(FramePtr<PointT> frame) override;

  void segmentation(const FrameConstPtr<PointT>& frame,
                    FramePtr<PointT>             dynamicFrame,
                    FramePtr<PointT>             staticFrame,
                    FramePtr<PointT>             staticFrameAdded,
                    FramePtr<PointT>             staticFrameRemoved) override;
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentationOPCGMMCenter.hpp>
