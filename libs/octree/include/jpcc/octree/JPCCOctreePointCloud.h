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
  using Frame    = jpcc::Frame<PointT>;
  using FramePtr = typename Frame::Ptr;

  JPCCOctreePointCloud(double resolution);

  void setFrame(FramePtr frame);

  void setFrame(BufferIndex bufferIndex, FramePtr frame);
};

}  // namespace jpcc::octree

#include <jpcc/octree/impl/JPCCOctreePointCloud.hpp>
