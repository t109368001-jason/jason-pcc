namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCSegmentationNone<PointT>::JPCCSegmentationNone(const JPCCSegmentationParameter& parameter,
                                                   const int                        startFrameNumber) :
    JPCCSegmentation<PointT>(parameter, startFrameNumber) {}

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
void JPCCSegmentationNone<PointT>::segmentation(const FrameConstPtr<PointT>& frame,
                                                const FramePtr<PointT>&      dynamicFrame,
                                                const FramePtr<PointT>&      staticFrame,
                                                const FramePtr<PointT>&      staticAddedFrame,
                                                const FramePtr<PointT>&      staticRemovedFrame) {
  pcl::copyPointCloud(*frame, *dynamicFrame);
}

}  // namespace jpcc::segmentation