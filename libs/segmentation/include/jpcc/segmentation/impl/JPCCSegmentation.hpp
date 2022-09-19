namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCSegmentation<PointT>::JPCCSegmentation(const JPCCSegmentationParameter& parameter, const int startFrameNumber) :
    parameter_(parameter), startFrameNumber_(startFrameNumber) {
  THROW_IF_NOT(this->startFrameNumber_ >= 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCSegmentation<PointT>::isThreadSafe() {
  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentation<PointT>::segmentation(const GroupOfFrame<PointT>& frames,
                                            const GroupOfFrame<PointT>& dynamicFrames,
                                            const GroupOfFrame<PointT>& staticFrames,
                                            const GroupOfFrame<PointT>& staticAddedFrames,
                                            const GroupOfFrame<PointT>& staticRemovedFrames,
                                            const bool                  parallel) {
  if (!parallel || !isThreadSafe()) {
    for (size_t i = 0; i < frames.size(); i++) {
      this->segmentation(frames.at(i), dynamicFrames.at(i), !staticFrames.empty() ? staticFrames.at(i) : nullptr,
                         !staticAddedFrames.empty() ? staticAddedFrames.at(i) : nullptr,
                         !staticRemovedFrames.empty() ? staticRemovedFrames.at(i) : nullptr);
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, frames.size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->segmentation(frames.at(i), dynamicFrames.at(i),
                                       !staticFrames.empty() ? staticFrames.at(i) : nullptr,
                                       !staticAddedFrames.empty() ? staticAddedFrames.at(i) : nullptr,
                                       !staticRemovedFrames.empty() ? staticRemovedFrames.at(i) : nullptr);
                  });
  }
}
}  // namespace jpcc::segmentation