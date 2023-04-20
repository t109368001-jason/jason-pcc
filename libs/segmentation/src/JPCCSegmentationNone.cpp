#include <jpcc/segmentation/JPCCSegmentationNone.h>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCSegmentationNone::JPCCSegmentationNone(const JPCCSegmentationParameter& parameter, const int startFrameNumber) :
    JPCCSegmentation(parameter, startFrameNumber) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCSegmentationNone::isThreadSafe() {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCSegmentationNone::isBuilt() const {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentationNone::appendTrainSamplesAndBuild(const FramePtr&                       frame,
                                                      const PclFramePtr<PointSegmentation>& pclFrame) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentationNone::segmentation(const GroupOfFrame&                       frames,
                                        const GroupOfPclFrame<PointSegmentation>& pclFrames,
                                        const GroupOfFrame&                       dynamicFrames,
                                        const GroupOfFrame&                       staticAddedFrames,
                                        const GroupOfFrame&                       staticRemovedFrames) {
  BOOST_THROW_EXCEPTION(std::logic_error("unsupported"));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentationNone::segmentation(IJPCCSegmentationContext& context, const size_t index) {
  context.getDynamicFrames()[index]->append(*context.getFrames()[index]);
}

}  // namespace jpcc::segmentation