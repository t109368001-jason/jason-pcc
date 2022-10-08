namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCContext<PointT>::JPCCContext(SegmentationType segmentationType, SegmentationOutputType segmentationOutputType) :
    segmentationType_(segmentationType), segmentationOutputType_(segmentationOutputType) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCContext<PointT>::clear() {
  pclFrames_.clear();
  dynamicPclFrames_.clear();
  staticPclFrames_.clear();
  staticAddedPclFrames_.clear();
  staticRemovedPclFrames_.clear();
  dynamicFrames_.clear();
  staticFrames_.clear();
  staticAddedFrames_.clear();
  staticRemovedFrames_.clear();
  dynamicEncodedBytes_.clear();
  staticEncodedBytes_.clear();
  staticAddedEncodedBytes_.clear();
  staticRemovedEncodedBytes_.clear();
  dynamicReconstructFrames_.clear();
  staticReconstructFrames_.clear();
  staticAddedReconstructFrames_.clear();
  staticRemovedReconstructFrames_.clear();
  dynamicReconstructPclFrames_.clear();
  staticReconstructPclFrames_.clear();
  staticAddedReconstructPclFrames_.clear();
  staticRemovedReconstructPclFrames_.clear();
  reconstructPclFrames_.clear();
}

}  // namespace jpcc