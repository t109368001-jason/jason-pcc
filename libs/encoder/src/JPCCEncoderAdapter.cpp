#include <jpcc/encoder/JPCCEncoderAdapter.h>

#include <jpcc/encoder/JPCCEncoderNone.h>
#if defined(HAVE_MPEG_PCC_TMC2)
#include <jpcc/encoder/JPCCEncoderTMC2.h>
#endif
#include <jpcc/encoder/JPCCEncoderTMC3.h>

namespace jpcc::encoder {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderAdapter::JPCCEncoderAdapter(const JPCCEncoderParameter& dynamicParameter,
                                       const JPCCEncoderParameter& staticParameter) {
  if (dynamicParameter.backendType == CoderBackendType::NONE) {
    dynamicEncoder_ = make_shared<JPCCEncoderNone>();
#if defined(HAVE_MPEG_PCC_TMC2)
  } else if (dynamicParameter.backendType == CoderBackendType::TMC2) {
    dynamicEncoder_ = make_shared<JPCCEncoderTMC2>(dynamicParameter.tmc2);
#endif
  } else if (dynamicParameter.backendType == CoderBackendType::TMC3) {
    dynamicEncoder_ = make_shared<JPCCEncoderTMC3>(dynamicParameter.tmc3);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
  if (staticParameter.backendType == CoderBackendType::NONE) {
    staticEncoder_        = make_shared<JPCCEncoderNone>();
    staticAddedEncoder_   = make_shared<JPCCEncoderNone>();
    staticRemovedEncoder_ = make_shared<JPCCEncoderNone>();
#if defined(HAVE_MPEG_PCC_TMC2)
  } else if (staticParameter.backendType == CoderBackendType::TMC2) {
    staticEncoder_        = make_shared<JPCCEncoderTMC2>(staticParameter.tmc2);
    staticAddedEncoder_   = make_shared<JPCCEncoderTMC2>(staticParameter.tmc2);
    staticRemovedEncoder_ = make_shared<JPCCEncoderTMC2>(staticParameter.tmc2);
#endif
  } else if (staticParameter.backendType == CoderBackendType::TMC3) {
    staticEncoder_        = make_shared<JPCCEncoderTMC3>(staticParameter.tmc3);
    staticAddedEncoder_   = make_shared<JPCCEncoderTMC3>(staticParameter.tmc3);
    staticRemovedEncoder_ = make_shared<JPCCEncoderTMC3>(staticParameter.tmc3);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderAdapter::convertToCoderType(JPCCContext& context, const bool parallel) {
  context.getDynamicContext().getCoderFrames().resize(context.getDynamicContext().getFrames().size());
  dynamicEncoder_->convertToCoderType(context.getDynamicFrames(), context.getDynamicContext().getCoderFrames(),
                                      parallel);
  if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC) {
    context.getStaticContext().getCoderFrames().resize(context.getStaticContext().getFrames().size());
    staticEncoder_->convertToCoderType(context.getStaticFrames(), context.getStaticContext().getCoderFrames(),
                                       parallel);
  } else if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    context.getStaticAddedContext().getCoderFrames().resize(context.getStaticAddedContext().getFrames().size());
    context.getStaticRemovedContext().getCoderFrames().resize(context.getStaticRemovedContext().getFrames().size());
    staticAddedEncoder_->convertToCoderType(context.getStaticAddedFrames(),
                                            context.getStaticAddedContext().getCoderFrames(), parallel);
    staticRemovedEncoder_->convertToCoderType(context.getStaticRemovedFrames(),
                                              context.getStaticRemovedContext().getCoderFrames(), parallel);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderAdapter::encode(JPCCContext& context, const bool parallel) {
  dynamicEncoder_->encode(context.getDynamicContext().getCoderFrames(), context.getDynamicContext().getOs(), parallel);
  if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC) {
    staticEncoder_->encode(context.getStaticContext().getCoderFrames(), context.getStaticContext().getOs(), parallel);
  } else if (context.getSegmentationOutputType() == SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
    staticAddedEncoder_->encode(context.getStaticAddedContext().getCoderFrames(),
                                context.getStaticAddedContext().getOs(), parallel);
    staticRemovedEncoder_->encode(context.getStaticRemovedContext().getCoderFrames(),
                                  context.getStaticRemovedContext().getOs(), parallel);
  }
}

}  // namespace jpcc::encoder