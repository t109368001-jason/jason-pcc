#pragma once

namespace jpcc::octree {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
JPCCOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::JPCCOctreePointCloud(double resolution) :
    OctreePointCloud(resolution) {
  this->defineBoundingBox(resolution * 2);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void JPCCOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::setFrame(FramePtr frame) {
  this->setInputCloud(frame);
  this->addPointsFromInputCloud();
}

}  // namespace jpcc::octree