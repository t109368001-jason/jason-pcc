#pragma once

#include <pcl/octree/octree_base.h>

#include <jpcc/common/Common.h>
#include <jpcc/octree/OctreeNBuf.h>
#include <jpcc/octree/OctreePointCloud.h>

namespace jpcc::octree {

template <typename PointT,
          typename LeafContainerT   = pcl::octree::OctreeContainerPointIndices,
          typename BranchContainerT = pcl::octree::OctreeContainerEmpty,
          typename OctreeT          = pcl::octree::OctreeBase<LeafContainerT, BranchContainerT>>
class JPCCOctreePointCloud : public OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT> {
 public:
  using OctreePointCloud = octree::OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>;

  using Frame         = jpcc::Frame<PointT>;
  using FramePtr      = typename Frame::Ptr;
  using FrameConstPtr = typename Frame::ConstPtr;

  JPCCOctreePointCloud(double resolution);

  void setFrame(FrameConstPtr frame);

  void setFrame(BufferIndex bufferIndex, FrameConstPtr frame);

  void addFrame(FrameConstPtr frame);

  void addFrame(BufferIndex bufferIndex, FrameConstPtr frame);
};

}  // namespace jpcc::octree

#include <jpcc/octree/impl/JPCCOctreePointCloud.hpp>
