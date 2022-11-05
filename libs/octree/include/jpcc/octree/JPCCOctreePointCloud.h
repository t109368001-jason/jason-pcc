#pragma once

#include <pcl/octree/octree_base.h>

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeContainerEditableIndex.h>
#include <jpcc/octree/OctreeNBuf.h>
#include <jpcc/octree/OctreePointCloud.h>
#include <jpcc/octree/type_traits.h>

namespace jpcc::octree {

template <typename PointT,
          typename LeafContainerT   = pcl::octree::OctreeContainerPointIndices,
          typename BranchContainerT = pcl::octree::OctreeContainerEmpty,
          typename OctreeT          = pcl::octree::OctreeBase<LeafContainerT, BranchContainerT>>
class JPCCOctreePointCloud : public OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT> {
 public:
  using Base = OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>;
  using Ptr  = shared_ptr<JPCCOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>>;

  using LeafNode   = typename Base::LeafNode;
  using BranchNode = typename Base::BranchNode;

  using OctreeKey = typename Base::OctreeKey;

  using PointCloudPtr = typename Base::PointCloudPtr;

  JPCCOctreePointCloud(double resolution);  // NOLINT(google-explicit-constructor)

  void setFrame(PclFrameConstPtr<PointT> frame);

  void setFrame(BufferIndex bufferIndex, PclFrameConstPtr<PointT> frame);

  void addFrame(PclFrameConstPtr<PointT> frame);

  void addFrame(BufferIndex bufferIndex, PclFrameConstPtr<PointT> frame);

  void deletePointFromCloud(const PointT& toDeletePoint, PointCloudPtr cloud);

 protected:
  void addPointIdx(UIndex point_idx_arg) override;

  void expandLeafNode(LeafNode*     leaf_node,
                      BranchNode*   parent_branch,
                      unsigned char child_idx,
                      UIndex        depth_mask) override;
};

}  // namespace jpcc::octree

#include <jpcc/octree/impl/JPCCOctreePointCloud.hpp>
