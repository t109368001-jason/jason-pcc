namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCSegmentation<PointT>::JPCCSegmentation(const JPCCSegmentationParameter& parameter) :
    parameter_(parameter), startFrameNumber_(0) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
size_t JPCCSegmentation<PointT>::getNTrain() const {
  return parameter_.nTrain;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentation<PointT>::setStartFrameNumber(const size_t startFrameNumber) {
  this->startFrameNumber_ = startFrameNumber;
}

}  // namespace jpcc::segmentation