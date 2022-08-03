#include <execution>

#include <jpcc/segmentation/JPCCSegmentationOPCGMMCenter.h>

namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCSegmentation<PointT>::JPCCSegmentation(const JPCCSegmentationParameter& parameter) :
    JPCCSegmentationBase<PointT>(parameter) {
  if (parameter.type == JPCCSegmentationOPCGMMCenter<PointT>::TYPE &&
      parameter.staticPointType == JPCCSegmentationOPCGMMCenter<PointT>::STATIC_POINT_TYPE) {
    backend_ = jpcc::make_shared<JPCCSegmentationOPCGMMCenter<PointT>>(this->parameter_);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentation<PointT>::appendTrainSamples(const GroupOfFrame<PointT>& groupOfFrame) {
  for (const auto& frame : groupOfFrame) { appendTrainSamples(frame); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentation<PointT>::setStartFrameNumber(const size_t startFrameNumber) {
  JPCCSegmentationBase<PointT>::setStartFrameNumber(startFrameNumber);
  backend_->setStartFrameNumber(startFrameNumber);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentation<PointT>::appendTrainSamples(FramePtr<PointT> frame) {
  backend_->appendTrainSamples(frame);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentation<PointT>::build() {
  backend_->build();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCSegmentation<PointT>::segmentation(const FrameConstPtr<PointT>& frame,
                                            FramePtr<PointT>             dynamicFrame,
                                            FramePtr<PointT>             staticFrame,
                                            FramePtr<PointT>             staticFrameAdded,
                                            FramePtr<PointT>             staticFrameRemoved) {
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