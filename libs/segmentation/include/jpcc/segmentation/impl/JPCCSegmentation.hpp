namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCSegmentation<PointT>::JPCCSegmentation(const JPCCSegmentationParameter& parameter) :
    parameter_(parameter), startFrameNumber_(0) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
int JPCCSegmentation<PointT>::getNTrain() const {
  return *std::max_element(parameter_.getNTrainVector().begin(), parameter_.getNTrainVector().end());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentation<PointT>::setStartFrameNumber(const size_t startFrameNumber) {
  this->startFrameNumber_ = startFrameNumber;
}

}  // namespace jpcc::segmentation