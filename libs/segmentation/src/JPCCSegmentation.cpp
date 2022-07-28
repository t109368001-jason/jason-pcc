#include <jpcc/segmentation/JPCCSegmentation.h>

#include <execution>

#include <jpcc/segmentation/JPCCSegmentationOPCGMMCemter.h>

using namespace std;
using namespace jpcc::octree;

namespace jpcc::segmentation {

JPCCSegmentation::JPCCSegmentation(const JPCCSegmentationParameter& parameter) : JPCCSegmentationBase(parameter) {
  if (parameter.type == JPCCSegmentationOPCGMMCemter::TYPE &&
      parameter.staticPointType == JPCCSegmentationOPCGMMCemter::STATIC_POINT_TYPE) {
    backend_ = jpcc::make_shared<JPCCSegmentationOPCGMMCemter>(parameter_);
  } else {
    BOOST_THROW_EXCEPTION(logic_error("unsupported staticPointType"));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentation::appendTrainSamples(const GroupOfFrame& groupOfFrame) {
  for (const auto& frame : groupOfFrame) { appendTrainSamples(frame); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentation::appendTrainSamples(FramePtr frame) { backend_->appendTrainSamples(frame); }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentation::build() { backend_->build(); }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCSegmentation::segmentation(const FrameConstPtr& frame,
                                    FramePtr             dynamicFrame,
                                    FramePtr             staticFrame,
                                    FramePtr             staticFrameAdded,
                                    FramePtr             staticFrameRemoved) {
  backend_->segmentation(frame, dynamicFrame, staticFrame, staticFrameAdded, staticFrameRemoved);
  if (dynamicFrame) {
    cout << "segmentation dynamic "
         << "frameNumber=" << dynamicFrame->header.seq << ", "
         << "points=" << dynamicFrame->size() << endl;
  }
  if (staticFrame) {
    cout << "segmentation static "
         << "frameNumber=" << staticFrame->header.seq << ", "
         << "points=" << staticFrame->size() << endl;
  }
  if (staticFrameAdded) {
    cout << "segmentation static added "
         << "frameNumber=" << staticFrameAdded->header.seq << ", "
         << "points=" << staticFrameAdded->size() << endl;
  }
  if (staticFrameRemoved) {
    cout << "segmentation static removed "
         << "frameNumber=" << staticFrameRemoved->header.seq << ", "
         << "points=" << staticFrameRemoved->size() << endl;
  }
}

}  // namespace jpcc::segmentation