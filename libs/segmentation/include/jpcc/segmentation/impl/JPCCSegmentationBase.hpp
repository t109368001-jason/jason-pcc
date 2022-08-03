namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCSegmentationBase<PointT>::JPCCSegmentationBase(const JPCCSegmentationParameter& parameter) :
    parameter_(parameter), startFrameNumber_(0) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
size_t JPCCSegmentationBase<PointT>::getNTrain() const {
  return parameter_.nTrain;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentationBase<PointT>::setStartFrameNumber(const size_t startFrameNumber) {
  this->startFrameNumber_ = startFrameNumber;
}

}  // namespace jpcc::segmentation