#include <jpcc/segmentation/JPCCSegmentation.h>

#include <execution>

#include <jpcc/segmentation/JPCCSegmentationOPCGMMCenter.h>

using namespace std;
using namespace jpcc::octree;

namespace jpcc::segmentation {

JPCCSegmentation::JPCCSegmentation(const JPCCSegmentationParameter& parameter) : JPCCSegmentationBase(parameter) {
  if (parameter.type == JPCCSegmentationOPCGMMCenter::TYPE &&
      parameter.staticPointType == JPCCSegmentationOPCGMMCenter::STATIC_POINT_TYPE) {
    backend_ = jpcc::make_shared<JPCCSegmentationOPCGMMCenter>(parameter_);
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
  if (dynamicFrame) {
    dynamicFrame->clear();
    dynamicFrame->header = frame->header;
  }
  if (staticFrame) {
    staticFrame->clear();
    staticFrame->header = frame->header;
  }
  if (staticFrameAdded) {
    staticFrameAdded->clear();
    staticFrameAdded->header = frame->header;
  }
  if (staticFrameRemoved) {
    staticFrameRemoved->clear();
    staticFrameRemoved->header = frame->header;
  }

  backend_->segmentation(frame, dynamicFrame, staticFrame, staticFrameAdded, staticFrameRemoved);

  if (dynamicFrame) {
    dynamicFrame->width  = dynamicFrame->size();
    dynamicFrame->height = 1;
    cout << "segmentation dynamic "
         << "frameNumber=" << dynamicFrame->header.seq << ", "
         << "points=" << dynamicFrame->size() << endl;
  }
  if (staticFrame) {
    staticFrame->width  = staticFrame->size();
    staticFrame->height = 1;
    cout << "segmentation static "
         << "frameNumber=" << staticFrame->header.seq << ", "
         << "points=" << staticFrame->size() << endl;
  }
  if (staticFrameAdded) {
    staticFrameAdded->width  = staticFrameAdded->size();
    staticFrameAdded->height = 1;
    cout << "segmentation static added "
         << "frameNumber=" << staticFrameAdded->header.seq << ", "
         << "points=" << staticFrameAdded->size() << endl;
  }
  if (staticFrameRemoved) {
    staticFrameRemoved->width  = staticFrameRemoved->size();
    staticFrameRemoved->height = 1;
    cout << "segmentation static removed "
         << "frameNumber=" << staticFrameRemoved->header.seq << ", "
         << "points=" << staticFrameRemoved->size() << endl;
  }
}

}  // namespace jpcc::segmentation