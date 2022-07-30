using namespace std;

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCSegmentationBase<PointT>::JPCCSegmentationBase(const JPCCSegmentationParameter& parameter) :
    parameter_(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
size_t JPCCSegmentationBase<PointT>::getNTrain() const {
  return parameter_.nTrain;
}

}  // namespace jpcc::segmentation