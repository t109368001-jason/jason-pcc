namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCSegmentationNone<PointT>::JPCCSegmentationNone(const JPCCSegmentationParameter& parameter,
                                                   const int                        startFrameNumber) :
    JPCCSegmentation<PointT>(parameter, startFrameNumber) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCSegmentationNone<PointT>::isThreadSafe() {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCSegmentationNone<PointT>::isBuilt() const {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentationNone<PointT>::appendTrainSamplesAndBuild(const FramePtr<PointT>& frame) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentationNone<PointT>::segmentation(IJPCCSegmentationContext<PointT>& context, const size_t index) {
  pcl::copyPointCloud(*context.getPclFrames()[index], *context.getDynamicPclFrames()[index]);
}

}  // namespace jpcc::segmentation