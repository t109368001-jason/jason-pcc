#include <jpcc/segmentation/JPCCSegmentation.h>

#include <execution>

#include <boost/range/counting_range.hpp>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCSegmentation::JPCCSegmentation(const JPCCSegmentationParameter& parameter, const int startFrameNumber) :
    parameter_(parameter), startFrameNumber_(startFrameNumber) {
  THROW_IF_NOT(this->startFrameNumber_ >= 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCSegmentation::isThreadSafe() { return false; }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentation::appendTrainSamplesAndBuild(IJPCCSegmentationContext& context, const bool parallel) {
  if (!parallel || !isThreadSafe()) {
    for (size_t i = 0; i < context.getFrames().size(); i++) {
      this->appendTrainSamplesAndBuild(context.getFrames()[i], context.getPclFrames()[i]);
    }
  } else {
    const auto range = boost::counting_range<size_t>(0, context.getFrames().size());
    std::for_each(std::execution::par, range.begin(), range.end(),
                  [&](const size_t& i) {  //
                    this->appendTrainSamplesAndBuild(context.getFrames()[i], context.getPclFrames()[i]);
                  });
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentation::segmentation(IJPCCSegmentationContext& context, const bool parallel) {
  context.getDynamicFrames().clear();
  context.getStaticFrames().clear();
  context.getStaticAddedFrames().clear();
  context.getStaticRemovedFrames().clear();
  context.getDynamicFrames().resize(context.getPclFrames().size());
  std::for_each(context.getDynamicFrames().begin(), context.getDynamicFrames().end(),
                [](auto& frame) { frame = jpcc::make_shared<Frame>(); });
  if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC) {
    context.getStaticFrames().resize(context.getPclFrames().size());
    std::for_each(context.getStaticFrames().begin(), context.getStaticFrames().end(),
                  [](auto& frame) { frame = jpcc::make_shared<Frame>(); });
  }
  if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    context.getStaticAddedFrames().resize(context.getPclFrames().size());
    context.getStaticRemovedFrames().resize(context.getPclFrames().size());
    std::for_each(context.getStaticAddedFrames().begin(), context.getStaticAddedFrames().end(),
                  [](auto& frame) { frame = jpcc::make_shared<Frame>(); });
    std::for_each(context.getStaticRemovedFrames().begin(), context.getStaticRemovedFrames().end(),
                  [](auto& frame) { frame = jpcc::make_shared<Frame>(); });
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