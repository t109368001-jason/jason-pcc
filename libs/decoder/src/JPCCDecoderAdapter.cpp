#include <jpcc/decoder/JPCCDecoderAdapter.h>

#include <jpcc/decoder/JPCCDecoderNone.h>
#include <jpcc/decoder/JPCCDecoderTMC2.h>
#include <jpcc/decoder/JPCCDecoderTMC3.h>

namespace jpcc::decoder {

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderAdapter::set(const JPCCContext& context) {
  if (context.getDynamicContext().getHeader().backendType == CoderBackendType::NONE) {
    dynamicDecoder_ = make_shared<JPCCDecoderNone>();
  } else if (context.getDynamicContext().getHeader().backendType == CoderBackendType::TMC2) {
    dynamicDecoder_ = make_shared<JPCCDecoderTMC2>();
  } else if (context.getDynamicContext().getHeader().backendType == CoderBackendType::TMC3) {
    dynamicDecoder_ = make_shared<JPCCDecoderTMC3>();
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
  if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC) {
    if (context.getStaticContext().getHeader().backendType == CoderBackendType::NONE) {
      staticDecoder_ = make_shared<JPCCDecoderNone>();
    } else if (context.getStaticContext().getHeader().backendType == CoderBackendType::TMC2) {
      staticDecoder_ = make_shared<JPCCDecoderTMC2>();
    } else if (context.getStaticContext().getHeader().backendType == CoderBackendType::TMC3) {
      staticDecoder_ = make_shared<JPCCDecoderTMC3>();
    } else {
      BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
    }
  } else if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    if (context.getStaticAddedContext().getHeader().backendType == CoderBackendType::NONE) {
      staticAddedDecoder_ = make_shared<JPCCDecoderNone>();
    } else if (context.getStaticAddedContext().getHeader().backendType == CoderBackendType::TMC2) {
      staticAddedDecoder_ = make_shared<JPCCDecoderTMC2>();
    } else if (context.getStaticAddedContext().getHeader().backendType == CoderBackendType::TMC3) {
      staticAddedDecoder_ = make_shared<JPCCDecoderTMC3>();
    } else {
      BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
    }
    if (context.getStaticRemovedContext().getHeader().backendType == CoderBackendType::NONE) {
      staticRemovedDecoder_ = make_shared<JPCCDecoderNone>();
    } else if (context.getStaticRemovedContext().getHeader().backendType == CoderBackendType::TMC2) {
      staticRemovedDecoder_ = make_shared<JPCCDecoderTMC2>();
    } else if (context.getStaticRemovedContext().getHeader().backendType == CoderBackendType::TMC3) {
      staticRemovedDecoder_ = make_shared<JPCCDecoderTMC3>();
    } else {
      BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
    }
  } else if (context.getSegmentationOutputType() != SegmentationOutputType::NONE) {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderAdapter::decode(JPCCContext& context, const size_t frameCount) {
  context.getDynamicContext().getCoderFrames().resize(frameCount);
  if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC) {
    context.getStaticContext().getCoderFrames().resize(frameCount);
  } else if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    context.getStaticAddedContext().getCoderFrames().resize(frameCount);
    context.getStaticRemovedContext().getCoderFrames().resize(frameCount);
  }
  for (Index i = 0; i < frameCount; i++) {
    dynamicDecoder_->decode(context.getDynamicContext().getIs(), context.getDynamicContext().getCoderFrames()[i]);
    if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC) {
      staticDecoder_->decode(context.getStaticContext().getIs(), context.getStaticContext().getCoderFrames()[i]);
    } else if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
      staticAddedDecoder_->decode(context.getStaticAddedContext().getIs(),
                                  context.getStaticAddedContext().getCoderFrames()[i]);
      staticRemovedDecoder_->decode(context.getStaticRemovedContext().getIs(),
                                    context.getStaticRemovedContext().getCoderFrames()[i]);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderAdapter::convertFromCoderType(JPCCContext& context, const bool parallel) {
  context.getDynamicContext().getFrames().resize(context.getDynamicContext().getCoderFrames().size());
  dynamicDecoder_->convertFromCoderType(context.getDynamicContext().getCoderFrames(),
                                        context.getDynamicContext().getFrames(), parallel);
  for (Index i = 0; i < context.getDynamicContext().getFrames().size(); i++) {
    context.getDynamicContext().getFrames()[i]->setFrameNumber(context.getStartFrameNumber() + i);
  }
  if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC) {
    context.getStaticContext().getFrames().resize(context.getStaticContext().getCoderFrames().size());
    staticDecoder_->convertFromCoderType(context.getStaticContext().getCoderFrames(),
                                         context.getStaticContext().getFrames(), parallel);
    for (Index i = 0; i < context.getStaticContext().getFrames().size(); i++) {
      context.getStaticContext().getFrames()[i]->setFrameNumber(context.getStartFrameNumber() + i);
    }
  } else if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    context.getStaticAddedContext().getFrames().resize(context.getStaticAddedContext().getCoderFrames().size());
    context.getStaticRemovedContext().getFrames().resize(context.getStaticRemovedContext().getCoderFrames().size());
    staticAddedDecoder_->convertFromCoderType(context.getStaticAddedContext().getCoderFrames(),
                                              context.getStaticAddedContext().getFrames(), parallel);
    staticRemovedDecoder_->convertFromCoderType(context.getStaticRemovedContext().getCoderFrames(),
                                                context.getStaticRemovedContext().getFrames(), parallel);
    for (Index i = 0; i < context.getStaticAddedContext().getFrames().size(); i++) {
      context.getStaticAddedContext().getFrames()[i]->setFrameNumber(context.getStartFrameNumber() + i);
    }
    for (Index i = 0; i < context.getStaticRemovedContext().getFrames().size(); i++) {
      context.getStaticRemovedContext().getFrames()[i]->setFrameNumber(context.getStartFrameNumber() + i);
    }
  }
}

}  // namespace jpcc::decoder