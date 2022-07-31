#pragma once

#include <pcl/octree/octree_base.h>

#include <jpcc/common/Common.h>
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

  using LeafNode   = typename Base::LeafNode;
  using BranchNode = typename Base::BranchNode;

  using OctreeKey = typename Base::OctreeKey;

  JPCCOctreePointCloud(double resolution);

  void setFrame(FrameConstPtr<PointT> frame);

  void setFrame(BufferIndex bufferIndex, FrameConstPtr<PointT> frame);

  void addFrame(FrameConstPtr<PointT> frame);

  void addFrame(BufferIndex bufferIndex, FrameConstPtr<PointT> frame);

 protected:
  void addPointIdx(uindex_t point_idx_arg) override;

  void expandLeafNode(LeafNode*     leaf_node,
                      BranchNode*   parent_branch,
                      unsigned char child_idx,
                      uindex_t      depth_mask) override;
};

}  // namespace jpcc::octree

#include <jpcc/octree/impl/JPCCOctreePointCloud.hpp>
