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
    dynamicDecoder_->decode(is, context.getCoderDynamicReconstructFrames()[i]);
    context.getDynamicReconstructFrames()[i]->setFrameNumber(context.getStartFrameNumber() + i);
    if (staticDecoder_) {
      staticDecoder_->decode(is, context.getCoderStaticReconstructFrames()[i]);
      context.getStaticReconstructFrames()[i]->setFrameNumber(context.getStartFrameNumber() + i);
    }
    if (staticAddedDecoder_ && staticRemovedDecoder_) {
      staticAddedDecoder_->decode(is, context.getCoderStaticAddedReconstructFrames()[i]);
      staticRemovedDecoder_->decode(is, context.getCoderStaticRemovedReconstructFrames()[i]);
      context.getStaticAddedReconstructFrames()[i]->setFrameNumber(context.getStartFrameNumber() + i);
      context.getStaticRemovedReconstructFrames()[i]->setFrameNumber(context.getStartFrameNumber() + i);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderAdapter::convertFromCoderType(IJPCCDecoderContext& context, const bool parallel) {
  context.getDynamicReconstructFrames().resize(context.getCoderDynamicReconstructFrames().size());
  context.getStaticReconstructFrames().resize(context.getCoderStaticReconstructFrames().size());
  context.getStaticAddedReconstructFrames().resize(context.getCoderStaticAddedReconstructFrames().size());
  context.getStaticRemovedReconstructFrames().resize(context.getCoderStaticRemovedReconstructFrames().size());
  if (dynamicDecoder_) {
    dynamicDecoder_->convertFromCoderType(context.getCoderDynamicReconstructFrames(),
                                          context.getDynamicReconstructFrames(), parallel);
  }
  if (staticDecoder_) {
    staticDecoder_->convertFromCoderType(context.getCoderStaticReconstructFrames(),
                                         context.getStaticReconstructFrames(), parallel);
  }
  if (staticAddedDecoder_) {
    staticAddedDecoder_->convertFromCoderType(context.getCoderStaticAddedReconstructFrames(),
                                              context.getStaticAddedReconstructFrames(), parallel);
  }
  if (staticRemovedDecoder_) {
    staticRemovedDecoder_->convertFromCoderType(context.getCoderStaticRemovedReconstructFrames(),
                                                context.getStaticRemovedReconstructFrames(), parallel);
  }
}

}  // namespace jpcc::decoder