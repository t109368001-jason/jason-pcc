#include <jpcc/segmentation/JPCCSegmentation.h>

#include <execution>

#include <jpcc/segmentation/JPCCSegmentationOPCGMMAdaptive.h>

using namespace std;
using namespace jpcc::octree;

namespace jpcc::segmentation {

JPCCSegmentation::JPCCSegmentation(const JPCCSegmentationParameter& parameter) : JPCCSegmentationBase(parameter) {
  if (parameter.type == JPCCSegmentationOPCGMMAdaptive::TYPE &&
      parameter.staticPointType == JPCCSegmentationOPCGMMAdaptive::STATIC_POINT_TYPE) {
    octree_ = jpcc::make_shared<JPCCSegmentationOPCGMMAdaptive>(parameter_);
  } else {
    BOOST_THROW_EXCEPTION(logic_error("unsupported staticPointType"));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentation::appendTrainSamples(const GroupOfFrame& groupOfFrame) {
  for (const auto& frame : groupOfFrame) { appendTrainSamples(frame); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentation::appendTrainSamples(FramePtr frame) { octree_->appendTrainSamples(frame); }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentation::build() { octree_->build(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentation::segmentation(const FrameConstPtr& frame, FramePtr dynamicFrame, FramePtr staticFrame) {
  octree_->segmentation(frame, dynamicFrame, staticFrame);
}

}  // namespace jpcc::segmentation