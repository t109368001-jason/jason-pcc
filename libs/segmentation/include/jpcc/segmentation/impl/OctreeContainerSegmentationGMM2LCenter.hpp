namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
OctreeContainerSegmentationGMM2LCenter<PointT>::OctreeContainerSegmentationGMM2LCenter() :
    OctreeContainerStaticFlag(), OctreeContainerGMM2L<PointT>() {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void OctreeContainerSegmentationGMM2LCenter<PointT>::reset() {
  OctreeContainerStaticFlag::reset();
  OctreeContainerGMM2L<PointT>::reset();
}

}  // namespace jpcc::segmentation