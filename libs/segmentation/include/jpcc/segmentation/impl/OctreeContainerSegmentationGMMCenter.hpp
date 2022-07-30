namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
OctreeContainerSegmentationGMMCenter<PointT>::OctreeContainerSegmentationGMMCenter() :
    OctreeContainerStaticFlag(), OctreeContainerGMM<PointT>() {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerSegmentationGMMCenter<PointT>::reset() {
  OctreeContainerStaticFlag::reset();
  OctreeContainerGMM<PointT>::reset();
}

}  // namespace jpcc::segmentation