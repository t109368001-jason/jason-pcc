namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCContext<PointT>::init(const size_t frameCount) {
  clear();
  resize(frameCount);
  std::for_each(pclFrames.begin(), pclFrames.end(), [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
  std::for_each(dynamicPclFrames.begin(), dynamicPclFrames.end(),
                [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
  std::for_each(dynamicReconstructPclFrames.begin(), dynamicReconstructPclFrames.end(),
                [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
  if (segmentationType != SegmentationType::NONE) {
    if (segmentationOutputType == SegmentationOutputType::DYNAMIC_STATIC) {  //
      std::for_each(staticPclFrames.begin(), staticPclFrames.end(),
                    [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
      std::for_each(staticReconstructPclFrames.begin(), staticReconstructPclFrames.end(),
                    [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
    }
    if (segmentationOutputType == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {  //
      std::for_each(staticAddedPclFrames.begin(), staticAddedPclFrames.end(),
                    [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
      std::for_each(staticRemovedPclFrames.begin(), staticRemovedPclFrames.end(),
                    [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
      std::for_each(staticAddedReconstructPclFrames.begin(), staticAddedReconstructPclFrames.end(),
                    [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
      std::for_each(staticRemovedReconstructPclFrames.begin(), staticRemovedReconstructPclFrames.end(),
                    [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
    }
  }
  std::for_each(reconstructPclFrames.begin(), reconstructPclFrames.end(),
                [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCContext<PointT>::resize(const size_t frameCount) {
  pclFrames.resize(frameCount);
  dynamicPclFrames.resize(frameCount);
  dynamicFrames.resize(frameCount);
  dynamicReconstructFrames.resize(frameCount);
  dynamicReconstructPclFrames.resize(frameCount);
  if (segmentationType != SegmentationType::NONE) {
    if (segmentationOutputType == SegmentationOutputType::DYNAMIC_STATIC) {  //
      staticPclFrames.resize(frameCount);
      staticFrames.resize(frameCount);
      staticReconstructFrames.resize(frameCount);
      staticReconstructPclFrames.resize(frameCount);
    }
    if (segmentationOutputType == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {  //
      staticAddedPclFrames.resize(frameCount);
      staticRemovedPclFrames.resize(frameCount);
      staticAddedFrames.resize(frameCount);
      staticRemovedFrames.resize(frameCount);
      staticAddedReconstructFrames.resize(frameCount);
      staticRemovedReconstructFrames.resize(frameCount);
      staticAddedReconstructPclFrames.resize(frameCount);
      staticRemovedReconstructPclFrames.resize(frameCount);
    }
  }
  reconstructPclFrames.resize(frameCount);
}

//////////////////////////////////////////////////////////////////////////////////////////////
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
  dynamicEncodedBytes.clear();
  staticEncodedBytes.clear();
  staticAddedEncodedBytes.clear();
  staticRemovedEncodedBytes.clear();
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