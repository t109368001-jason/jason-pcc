#include <jpcc/decoder/JPCCDecoderAdapter.h>

#include "../include/jpcc/decoder/JPCCDecoderNone.h"
#include "../include/jpcc/decoder/JPCCDecoderTMC3.h"

namespace jpcc::decoder {

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderAdapter::set(JPCCHeader header) {
  if (header.dynamicBackendType == CoderBackendType::NONE) {
    dynamicDecoder_ = make_shared<JPCCDecoderNone>();
  } else if (header.dynamicBackendType == CoderBackendType::TMC3) {
    dynamicDecoder_ = make_shared<JPCCDecoderTMC3>();
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
  if (header.segmentationOutputType == SegmentationOutputType::DYNAMIC_STATIC) {
    if (header.staticBackendType == CoderBackendType::NONE) {
      staticDecoder_ = make_shared<JPCCDecoderNone>();
    } else if (header.staticBackendType == CoderBackendType::TMC3) {
      staticDecoder_ = make_shared<JPCCDecoderTMC3>();
    } else {
      BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
    }
  } else if (header.segmentationOutputType == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    if (header.staticBackendType == CoderBackendType::NONE) {
      staticAddedDecoder_   = make_shared<JPCCDecoderNone>();
      staticRemovedDecoder_ = make_shared<JPCCDecoderNone>();
    } else if (header.staticBackendType == CoderBackendType::TMC3) {
      staticAddedDecoder_   = make_shared<JPCCDecoderTMC3>();
      staticRemovedDecoder_ = make_shared<JPCCDecoderTMC3>();
    } else {
      BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
    }
  } else if (header.segmentationType != SegmentationType::NONE) {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderAdapter::decode(std::istream& is, IJPCCDecoderContext& context, const size_t frameCount) {
  context.getDynamicReconstructFrames().resize(frameCount);
  if (staticDecoder_) { context.getStaticReconstructFrames().resize(frameCount); }
  if (staticAddedDecoder_ && staticRemovedDecoder_) {
    context.getStaticAddedReconstructFrames().resize(frameCount);
    context.getStaticRemovedReconstructFrames().resize(frameCount);
  }
  for (Index i = 0; i < frameCount; i++) {
    dynamicDecoder_->decode(is, context.getDynamicReconstructFrames()[i]);
    context.getDynamicReconstructFrames()[i]->getFrameNumber() = context.getStartFrameNumber() + i;
    if (staticDecoder_) {
      staticDecoder_->decode(is, context.getStaticReconstructFrames()[i]);
      context.getStaticReconstructFrames()[i]->getFrameNumber() = context.getStartFrameNumber() + i;
    }
    if (staticAddedDecoder_ && staticRemovedDecoder_) {
      staticAddedDecoder_->decode(is, context.getStaticAddedReconstructFrames()[i]);
      staticRemovedDecoder_->decode(is, context.getStaticRemovedReconstructFrames()[i]);
      context.getStaticAddedReconstructFrames()[i]->getFrameNumber()   = context.getStartFrameNumber() + i;
      context.getStaticRemovedReconstructFrames()[i]->getFrameNumber() = context.getStartFrameNumber() + i;
    }
  }
}

}  // namespace jpcc::decoder