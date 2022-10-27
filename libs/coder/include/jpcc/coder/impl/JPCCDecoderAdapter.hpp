#include <jpcc/coder/JPCCDecoderNone.h>
#include <jpcc/coder/JPCCDecoderTMC3.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderAdapter<PointT>::setBackendType(JPCCHeader header) {
  if (header.dynamicBackendType == CoderBackendType::NONE) {
    dynamicDecoder_ = make_shared<JPCCDecoderNone<PointT>>();
  } else if (header.dynamicBackendType == CoderBackendType::TMC3) {
    dynamicDecoder_ = make_shared<JPCCDecoderTMC3<PointT>>();
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
  if (header.segmentationOutputType == SegmentationOutputType::DYNAMIC_STATIC) {
    if (header.staticBackendType == CoderBackendType::NONE) {
      staticDecoder_ = make_shared<JPCCDecoderNone<PointT>>();
    } else if (header.staticBackendType == CoderBackendType::TMC3) {
      staticDecoder_ = make_shared<JPCCDecoderTMC3<PointT>>();
    } else {
      BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
    }
  } else if (header.segmentationOutputType == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    if (header.staticBackendType == CoderBackendType::NONE) {
      staticAddedDecoder_   = make_shared<JPCCDecoderNone<PointT>>();
      staticRemovedDecoder_ = make_shared<JPCCDecoderNone<PointT>>();
    } else if (header.staticBackendType == CoderBackendType::TMC3) {
      staticAddedDecoder_   = make_shared<JPCCDecoderTMC3<PointT>>();
      staticRemovedDecoder_ = make_shared<JPCCDecoderTMC3<PointT>>();
    } else {
      BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
    }
  } else if (header.segmentationType != SegmentationType::NONE) {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderAdapter<PointT>::decode(std::istream&                is,
                                        IJPCCDecoderContext<PointT>& context,
                                        const size_t                 frameCount,
                                        const bool                   parallel) {
  context.getDynamicReconstructFrames().resize(frameCount);
  if (staticDecoder_) { context.getStaticReconstructFrames().resize(frameCount); }
  if (staticAddedDecoder_ && staticRemovedDecoder_) {
    context.getStaticAddedReconstructFrames().resize(frameCount);
    context.getStaticRemovedReconstructFrames().resize(frameCount);
  }
  for (size_t i = 0; i < frameCount; i++) {
    dynamicDecoder_->decode(is, context.getDynamicReconstructFrames()[i]);
    if (staticDecoder_) { staticDecoder_->decode(is, context.getStaticReconstructFrames()[i]); }
    if (staticAddedDecoder_ && staticRemovedDecoder_) {
      staticAddedDecoder_->decode(is, context.getStaticAddedReconstructFrames()[i]);
      staticRemovedDecoder_->decode(is, context.getStaticRemovedReconstructFrames()[i]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderAdapter<PointT>::convertToPCL(IJPCCDecoderContext<PointT>& context, const bool parallel) {
  dynamicDecoder_->convertToPCL(context.getDynamicReconstructFrames(), context.getDynamicReconstructPclFrames(),
                                parallel);
  if (staticDecoder_) {
    staticDecoder_->convertToPCL(context.getStaticReconstructFrames(), context.getStaticReconstructPclFrames(),
                                 parallel);
  }
  if (staticAddedDecoder_ && staticRemovedDecoder_) {
    staticAddedDecoder_->convertToPCL(context.getStaticAddedReconstructFrames(),
                                      context.getStaticAddedReconstructPclFrames(), parallel);
    staticRemovedDecoder_->convertToPCL(context.getStaticRemovedReconstructFrames(),
                                        context.getStaticRemovedReconstructPclFrames(), parallel);
  }
}

}  // namespace jpcc::coder