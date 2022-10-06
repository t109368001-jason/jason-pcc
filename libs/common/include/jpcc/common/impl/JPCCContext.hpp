namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCContext<PointT>::JPCCContext(SegmentationType segmentationType, SegmentationOutputType segmentationOutputType) :
    segmentationType_(segmentationType), segmentationOutputType_(segmentationOutputType) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCContext<PointT>::init(const size_t frameCount) {
  clear();
  resize(frameCount);
  std::for_each(pclFrames_.begin(), pclFrames_.end(), [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
  std::for_each(dynamicPclFrames_.begin(), dynamicPclFrames_.end(),
                [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
  std::for_each(dynamicReconstructPclFrames_.begin(), dynamicReconstructPclFrames_.end(),
                [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
  if (segmentationType_ != SegmentationType::NONE) {
    if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC) {  //
      std::for_each(staticPclFrames_.begin(), staticPclFrames_.end(),
                    [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
      std::for_each(staticReconstructPclFrames_.begin(), staticReconstructPclFrames_.end(),
                    [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
    }
    if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {  //
      std::for_each(staticAddedPclFrames_.begin(), staticAddedPclFrames_.end(),
                    [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
      std::for_each(staticRemovedPclFrames_.begin(), staticRemovedPclFrames_.end(),
                    [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
      std::for_each(staticAddedReconstructPclFrames_.begin(), staticAddedReconstructPclFrames_.end(),
                    [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
      std::for_each(staticRemovedReconstructPclFrames_.begin(), staticRemovedReconstructPclFrames_.end(),
                    [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
    }
  }
  std::for_each(reconstructPclFrames_.begin(), reconstructPclFrames_.end(),
                [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCContext<PointT>::resize(const size_t frameCount) {
  pclFrames_.resize(frameCount);
  dynamicPclFrames_.resize(frameCount);
  dynamicFrames_.resize(frameCount);
  dynamicReconstructFrames_.resize(frameCount);
  dynamicReconstructPclFrames_.resize(frameCount);
  if (segmentationType_ != SegmentationType::NONE) {
    if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC) {  //
      staticPclFrames_.resize(frameCount);
      staticFrames_.resize(frameCount);
      staticReconstructFrames_.resize(frameCount);
      staticReconstructPclFrames_.resize(frameCount);
    }
    if (segmentationOutputType_ == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {  //
      staticAddedPclFrames_.resize(frameCount);
      staticRemovedPclFrames_.resize(frameCount);
      staticAddedFrames_.resize(frameCount);
      staticRemovedFrames_.resize(frameCount);
      staticAddedReconstructFrames_.resize(frameCount);
      staticRemovedReconstructFrames_.resize(frameCount);
      staticAddedReconstructPclFrames_.resize(frameCount);
      staticRemovedReconstructPclFrames_.resize(frameCount);
    }
  }
  reconstructPclFrames_.resize(frameCount);
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