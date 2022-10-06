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
void JPCCSegmentation<PointT>::appendTrainSamplesAndBuild(const GroupOfFrame<PointT>& frames, const bool parallel) {
  if (!parallel || !isThreadSafe()) {
    for (const auto& frame : frames) { this->appendTrainSamplesAndBuild(frame); }
  } else {
    std::for_each(std::execution::par, frames.begin(), frames.end(),
                  [&](const auto& frame) {  //
                    this->appendTrainSamplesAndBuild(frame);
                  });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentation<PointT>::segmentation(IJPCCSegmentationContext<PointT>& context, const bool parallel) {
  context.getDynamicPclFrames().clear();
  context.getStaticPclFrames().clear();
  context.getStaticAddedPclFrames().clear();
  context.getStaticRemovedPclFrames().clear();
  context.getDynamicPclFrames().resize(context.getPclFrames().size());
  std::for_each(context.getDynamicPclFrames().begin(), context.getDynamicPclFrames().end(),
                [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
  if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC) {
    context.getStaticPclFrames().resize(context.getPclFrames().size());
    std::for_each(context.getStaticPclFrames().begin(), context.getStaticPclFrames().end(),
                  [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
  }
  if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    context.getStaticAddedPclFrames().resize(context.getPclFrames().size());
    context.getStaticRemovedPclFrames().resize(context.getPclFrames().size());
    std::for_each(context.getStaticAddedPclFrames().begin(), context.getStaticAddedPclFrames().end(),
                  [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
    std::for_each(context.getStaticRemovedPclFrames().begin(), context.getStaticRemovedPclFrames().end(),
                  [](auto& frame) { frame = jpcc::make_shared<Frame<PointT>>(); });
  }
  if (!parallel || !isThreadSafe()) {
    for (size_t i = 0; i < context.getPclFrames().size(); i++) { this->segmentation(context, i); }
  } else {
    const auto range = boost::counting_range<size_t>(0, context.getPclFrames().size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->segmentation(context, i);
                  });
  }
}

}  // namespace jpcc::segmentation