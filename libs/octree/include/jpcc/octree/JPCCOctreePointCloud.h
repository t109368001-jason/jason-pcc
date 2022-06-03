#pragma once

#include <pcl/octree/octree_base.h>

#include <jpcc/octree/OctreePointCloud.h>

namespace jpcc::octree {

template <typename PointT,
          typename LeafContainerT   = pcl::octree::OctreeContainerPointIndices,
          typename BranchContainerT = pcl::octree::OctreeContainerEmpty,
          typename OctreeT          = pcl::octree::OctreeBase<LeafContainerT, BranchContainerT>>
class JPCCOctreePointCloud : public OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT> {
 public:
  JPCCOctreePointCloud(double resolution);
};

}  // namespace jpcc::octree

#include <jpcc/octree/impl/JPCCOctreePointCloud.hpp>
