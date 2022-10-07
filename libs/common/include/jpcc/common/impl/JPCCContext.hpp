namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCContext<PointT>::JPCCContext(SegmentationType segmentationType, SegmentationOutputType segmentationOutputType) :
    segmentationType_(segmentationType), segmentationOutputType_(segmentationOutputType) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCContext<PointT>::copyNormalToReconstruct() {
  for (size_t i = 0; i < dynamicReconstructPclFrames_.size(); i++) {
    FramePtr<PointT>& dynamicReconstructPclFrame = dynamicReconstructPclFrames_[i];
    FramePtr<PointT>& dynamicPclFrame            = dynamicPclFrames_[i];
    for (size_t j = 0; j < dynamicReconstructPclFrame->points.size(); j++) {
      (*dynamicReconstructPclFrame)[j].normal_x  = (*dynamicPclFrame)[j].normal_x;
      (*dynamicReconstructPclFrame)[j].normal_y  = (*dynamicPclFrame)[j].normal_y;
      (*dynamicReconstructPclFrame)[j].normal_z  = (*dynamicPclFrame)[j].normal_z;
      (*dynamicReconstructPclFrame)[j].curvature = (*dynamicPclFrame)[j].curvature;
    }
  }
  if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC) {
    for (size_t i = 0; i < staticReconstructPclFrames_.size(); i++) {
      FramePtr<PointT>& staticReconstructPclFrame = staticReconstructPclFrames_[i];
      FramePtr<PointT>& staticPclFrames           = staticPclFrames_[i];
      for (size_t j = 0; j < staticReconstructPclFrame->points.size(); j++) {
        (*staticReconstructPclFrame)[j].normal_x  = (*staticPclFrames)[j].normal_x;
        (*staticReconstructPclFrame)[j].normal_y  = (*staticPclFrames)[j].normal_y;
        (*staticReconstructPclFrame)[j].normal_z  = (*staticPclFrames)[j].normal_z;
        (*staticReconstructPclFrame)[j].curvature = (*staticPclFrames)[j].curvature;
      }
    }
  }
  if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    for (size_t i = 0; i < staticAddedReconstructPclFrames_.size(); i++) {
      FramePtr<PointT>& staticAddedReconstructPclFrame = staticAddedReconstructPclFrames_[i];
      FramePtr<PointT>& staticAddedPclFrame            = staticAddedPclFrames_[i];
      for (size_t j = 0; j < staticAddedReconstructPclFrame->points.size(); j++) {
        (*staticAddedReconstructPclFrame)[j].normal_x  = (*staticAddedPclFrame)[j].normal_x;
        (*staticAddedReconstructPclFrame)[j].normal_y  = (*staticAddedPclFrame)[j].normal_y;
        (*staticAddedReconstructPclFrame)[j].normal_z  = (*staticAddedPclFrame)[j].normal_z;
        (*staticAddedReconstructPclFrame)[j].curvature = (*staticAddedPclFrame)[j].curvature;
      }
    }
    for (size_t i = 0; i < staticRemovedReconstructPclFrames_.size(); i++) {
      FramePtr<PointT>& staticRemovedReconstructPclFrame = staticRemovedReconstructPclFrames_[i];
      FramePtr<PointT>& staticRemovedPclFrame            = staticRemovedPclFrames_[i];
      for (size_t j = 0; j < staticRemovedReconstructPclFrame->points.size(); j++) {
        (*staticRemovedReconstructPclFrame)[j].normal_x  = (*staticRemovedPclFrame)[j].normal_x;
        (*staticRemovedReconstructPclFrame)[j].normal_y  = (*staticRemovedPclFrame)[j].normal_y;
        (*staticRemovedReconstructPclFrame)[j].normal_z  = (*staticRemovedPclFrame)[j].normal_z;
        (*staticRemovedReconstructPclFrame)[j].curvature = (*staticRemovedPclFrame)[j].curvature;
      }
    }
  }
}

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