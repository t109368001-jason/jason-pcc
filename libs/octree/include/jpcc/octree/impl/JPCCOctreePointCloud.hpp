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
  if constexpr (is_n_buf_octree_v<OctreeT>) {
    this->deleteBuffer(this->getBufferIndex());
    this->setInputCloud(frame);
    this->addPointsFromInputCloud();
  } else {
    this->setInputCloud(frame);
    this->addPointsFromInputCloud();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void JPCCOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::setFrame(BufferIndex bufferIndex,
                                                                                       FramePtr    frame) {
  if constexpr (is_n_buf_octree_v<OctreeT>) {
    this->switchBuffers(bufferIndex);
    setFrame(frame);
  } else {
    static_assert(false, "setFrame only support for OctreeNBuf, please use setFrame");
  }
}

}  // namespace jpcc::octree