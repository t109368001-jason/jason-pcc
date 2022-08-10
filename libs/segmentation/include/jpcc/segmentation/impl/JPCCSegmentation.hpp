namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCSegmentation<PointT>::JPCCSegmentation(const JPCCSegmentationParameter& parameter, const int startFrameNumber) :
    parameter_(parameter), startFrameNumber_(startFrameNumber), built_(false) {
  THROW_IF_NOT(this->startFrameNumber_ >= 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCSegmentation<PointT>::isBuilt() const {
  return built_;
}

}  // namespace jpcc::segmentation