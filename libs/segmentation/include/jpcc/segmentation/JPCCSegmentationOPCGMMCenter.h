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

  using OctreeNode = pcl::octree::OctreeNode;
  using LeafNode   = typename Base::LeafNode;
  using BranchNode = typename Base::BranchNode;

  using OctreeKey = typename Base::OctreeKey;

 protected:
  std::vector<float>     alternateCentroids_;
  std::array<bool, SIZE> builtVector;

 public:
  JPCCSegmentationOPCGMMCenter(const JPCCSegmentationParameter& parameter, int startFrameNumber);

  [[nodiscard]] bool isBuilt() const override;

  void appendTrainSamplesAndBuild(const FramePtr<PointT>& frame) override;

  void segmentation(const FrameConstPtr<PointT>& frame,
                    const FramePtr<PointT>&      dynamicFrame,
                    const FramePtr<PointT>&      staticFrame,
                    const FramePtr<PointT>&      staticFrameAdded,
                    const FramePtr<PointT>&      staticFrameRemoved) override;

  void appendTrainSamples(const FramePtr<PointT>& frame);

  void addTrainSampleAndResetLastPointRecursive(const FramePtr<PointT>& frame, const BranchNode* branchNode);

  void build(const FramePtr<PointT>& frame);

  void buildRecursive(const FramePtr<PointT>& frame, size_t index, const BranchNode* branchNode);

  void segmentationRecursive(const FrameConstPtr<PointT>& frame,
                             const FramePtr<PointT>&      dynamicFrame,
                             const FramePtr<PointT>&      staticFrame,
                             const FramePtr<PointT>&      staticFrameAdded,
                             const FramePtr<PointT>&      staticFrameRemoved,
                             OctreeKey&                   key,
                             const BranchNode*            branchNode);
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentationOPCGMMCenter.hpp>
