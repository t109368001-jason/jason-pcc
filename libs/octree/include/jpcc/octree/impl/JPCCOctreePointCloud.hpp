#pragma once

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
JPCCOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::JPCCOctreePointCloud(double resolution) :
    OctreePointCloud(resolution) {}

}  // namespace jpcc::octree