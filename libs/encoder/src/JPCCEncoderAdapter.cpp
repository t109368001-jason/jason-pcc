#include <jpcc/encoder/JPCCEncoderAdapter.h>

#include "../include/jpcc/encoder/JPCCEncoderNone.h"
#include "../include/jpcc/encoder/JPCCEncoderTMC3.h"

namespace jpcc::encoder {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderAdapter::JPCCEncoderAdapter(const JPCCEncoderParameter& dynamicParameter,
                                       const JPCCEncoderParameter& staticParameter) {
  if (dynamicParameter.backendType == CoderBackendType::NONE) {
    dynamicEncoder_ = make_shared<JPCCEncoderNone>();
  } else if (dynamicParameter.backendType == CoderBackendType::TMC3) {
    dynamicEncoder_ = make_shared<JPCCEncoderTMC3>(dynamicParameter.tmc3);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
  if (staticParameter.backendType == CoderBackendType::NONE) {
    staticEncoder_        = make_shared<JPCCEncoderNone>();
    staticAddedEncoder_   = make_shared<JPCCEncoderNone>();
    staticRemovedEncoder_ = make_shared<JPCCEncoderNone>();
  } else if (staticParameter.backendType == CoderBackendType::TMC3) {
    staticEncoder_        = make_shared<JPCCEncoderTMC3>(staticParameter.tmc3);
    staticAddedEncoder_   = make_shared<JPCCEncoderTMC3>(staticParameter.tmc3);
    staticRemovedEncoder_ = make_shared<JPCCEncoderTMC3>(staticParameter.tmc3);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderAdapter::convertToCoderType(IJPCCEncoderContext& context, const bool parallel) {
  context.getCoderDynamicFrames().resize(context.getDynamicFrames().size());
  context.getCoderStaticFrames().resize(context.getStaticFrames().size());
  context.getCoderStaticAddedFrames().resize(context.getStaticAddedFrames().size());
  context.getCoderStaticRemovedFrames().resize(context.getStaticRemovedFrames().size());
  if (dynamicEncoder_) {
    dynamicEncoder_->convertToCoderType(context.getDynamicFrames(), context.getCoderDynamicFrames(), parallel);
  }
  if (staticEncoder_) {
    staticEncoder_->convertToCoderType(context.getStaticFrames(), context.getCoderStaticFrames(), parallel);
  }
  if (staticAddedEncoder_) {
    staticAddedEncoder_->convertToCoderType(context.getStaticAddedFrames(), context.getCoderStaticAddedFrames(),
                                            parallel);
  }
  if (staticRemovedEncoder_) {
    staticRemovedEncoder_->convertToCoderType(context.getStaticRemovedFrames(), context.getCoderStaticRemovedFrames(),
                                              parallel);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderAdapter::encode(IJPCCEncoderContext& context, const bool parallel) {
  context.getDynamicEncodedBytesVector().resize(context.getDynamicFrames().size());
  context.getStaticEncodedBytesVector().resize(context.getStaticFrames().size());
  context.getStaticAddedEncodedBytesVector().resize(context.getStaticAddedFrames().size());
  context.getStaticRemovedEncodedBytesVector().resize(context.getStaticRemovedFrames().size());
  if (dynamicEncoder_) {
    dynamicEncoder_->encode(context.getCoderDynamicFrames(), context.getDynamicEncodedBytesVector(), parallel);
  }
  if (staticEncoder_) {
    staticEncoder_->encode(context.getCoderStaticFrames(), context.getStaticEncodedBytesVector(), parallel);
  }
  if (staticAddedEncoder_) {
    staticAddedEncoder_->encode(context.getCoderStaticAddedFrames(), context.getStaticAddedEncodedBytesVector(),
                                parallel);
  }
  if (staticRemovedEncoder_) {
    staticRemovedEncoder_->encode(context.getCoderStaticRemovedFrames(), context.getStaticRemovedEncodedBytesVector(),
                                  parallel);
  }
}

}  // namespace jpcc::encoder