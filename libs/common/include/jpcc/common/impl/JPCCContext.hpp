namespace jpcc {

template <typename PointT>
void JPCCContext<PointT>::init(size_t frameCount) {
  pclFrames.resize(frameCount);
  dynamicPclFrames.resize(frameCount);
  dynamicFrames.resize(frameCount);
  dynamicEncodedFramesBytes.resize(frameCount);
  dynamicReconstructFrames.resize(frameCount);
  dynamicReconstructPclFrames.resize(frameCount);
  if (segmentationOutputType == SegmentationOutputType::DYNAMIC_STATIC) {  //
    staticPclFrames.resize(frameCount);
    staticFrames.resize(frameCount);
    staticEncodedFramesBytes.resize(frameCount);
    staticReconstructFrames.resize(frameCount);
    staticReconstructPclFrames.resize(frameCount);
  }
  if (segmentationOutputType == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {  //
    staticAddedPclFrames.resize(frameCount);
    staticRemovedPclFrames.resize(frameCount);
    staticAddedFrames.resize(frameCount);
    staticRemovedFrames.resize(frameCount);
    staticAddedEncodedFramesBytes.resize(frameCount);
    staticRemovedEncodedFramesBytes.resize(frameCount);
    staticAddedReconstructFrames.resize(frameCount);
    staticRemovedReconstructFrames.resize(frameCount);
    staticAddedReconstructPclFrames.resize(frameCount);
    staticRemovedReconstructPclFrames.resize(frameCount);
  }
  reconstructPclFrames.resize(frameCount);
}

template <typename PointT>
void JPCCContext<PointT>::clear() {
  pclFrames.clear();
  dynamicPclFrames.clear();
  staticPclFrames.clear();
  staticAddedPclFrames.clear();
  staticRemovedPclFrames.clear();
  dynamicFrames.clear();
  staticFrames.clear();
  staticAddedFrames.clear();
  staticRemovedFrames.clear();
  dynamicEncodedFramesBytes.clear();
  staticEncodedFramesBytes.clear();
  staticAddedEncodedFramesBytes.clear();
  staticRemovedEncodedFramesBytes.clear();
  dynamicReconstructFrames.clear();
  staticReconstructFrames.clear();
  staticAddedReconstructFrames.clear();
  staticRemovedReconstructFrames.clear();
  dynamicReconstructPclFrames.clear();
  staticReconstructPclFrames.clear();
  staticAddedReconstructPclFrames.clear();
  staticRemovedReconstructPclFrames.clear();
  reconstructPclFrames.clear();
}

}  // namespace jpcc