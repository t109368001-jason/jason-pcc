#pragma once

#include <array>
#include <set>

#include <jpcc/common/Common.h>

#include <jpcc/octree/JPCCOctreePointCloud.h>
#include <jpcc/segmentation/JPCCSegmentation.h>

namespace jpcc::segmentation {

template <typename LeafContainerT>
class JPCCSegmentationOPCGMMCenter : virtual public JPCCSegmentation,
                                     virtual public octree::JPCCOctreePointCloud<PointSegmentation, LeafContainerT> {
 public:
  static constexpr int SIZE = LeafContainerT::SIZE;

  using Ptr  = shared_ptr<JPCCSegmentationOPCGMMCenter>;
  using Base = octree::JPCCOctreePointCloud<PointSegmentation, LeafContainerT>;

  using PointT = PointSegmentation;

  using OctreeNode = pcl::octree::OctreeNode;
  using LeafNode   = typename Base::LeafNode;
  using BranchNode = typename Base::BranchNode;

  using OctreeKey = typename Base::OctreeKey;

 protected:
  std::set<Intensity>    alternateCentroids_;
  std::array<bool, SIZE> builtVector;

 public:
  JPCCSegmentationOPCGMMCenter(const JPCCSegmentationParameter& parameter, int startFrameNumber);

  [[nodiscard]] bool isBuilt() const override;

  void appendTrainSamplesAndBuild(const FramePtr& frame, const PclFramePtr<PointT>& pclFrame) override;

  void segmentation(IJPCCSegmentationContext& context, size_t index) override;

  void appendTrainSamples(const FramePtr& frame);

  void addTrainSampleAndResetLastPointRecursive(const FramePtr& frame, const BranchNode* branchNode);

  void build(const FramePtr& frame);

  void buildRecursive(const FramePtr& frame, size_t index, const BranchNode* branchNode);

  void segmentationRecursive(const FrameConstPtr& frame,
                             const FramePtr&      dynamicFrame,
                             const FramePtr&      staticFrame,
                             const FramePtr&      staticAddedFrame,
                             const FramePtr&      staticRemovedFrame,
                             OctreeKey&           key,
                             const BranchNode*    branchNode);
};

}  // namespace jpcc::segmentation

#include <jpcc/segmentation/impl/JPCCSegmentationOPCGMMCenter.hpp>
