#include <jpcc/encoder/JPCCEncoderAdapter.h>

#include "../include/jpcc/encoder/JPCCEncoderNone.h"
#include "../include/jpcc/encoder/JPCCEncoderTMC3.h"

namespace jpcc::encoder {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderAdapter::JPCCEncoderAdapter(const JPCCEncoderParameter& dynamicParameter,
                                       const JPCCEncoderParameter& staticParameter) {
  if (dynamicParameter.backendType == CoderBackendType::NONE) {
    dynamicEncoder_ = make_shared<JPCCEncoderNone>(dynamicParameter);
  } else if (dynamicParameter.backendType == CoderBackendType::TMC3) {
    dynamicEncoder_ = make_shared<JPCCEncoderTMC3>(dynamicParameter);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
  if (staticParameter.backendType == CoderBackendType::NONE) {
    staticEncoder_        = make_shared<JPCCEncoderNone>(staticParameter);
    staticAddedEncoder_   = make_shared<JPCCEncoderNone>(staticParameter);
    staticRemovedEncoder_ = make_shared<JPCCEncoderNone>(staticParameter);
  } else if (staticParameter.backendType == CoderBackendType::TMC3) {
    staticEncoder_        = make_shared<JPCCEncoderTMC3>(staticParameter);
    staticAddedEncoder_   = make_shared<JPCCEncoderTMC3>(staticParameter);
    staticRemovedEncoder_ = make_shared<JPCCEncoderTMC3>(staticParameter);
  } else {
    BOOST_THROW_EXCEPTION(std::logic_error("unsupported staticPointType"));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderAdapter::encode(IJPCCEncoderContext& context, const bool parallel) {
  context.getDynamicEncodedBytesVector().resize(context.getDynamicFrames().size());
  context.getStaticEncodedBytesVector().resize(context.getStaticFrames().size());
  context.getStaticAddedEncodedBytesVector().resize(context.getStaticAddedFrames().size());
  context.getStaticRemovedEncodedBytesVector().resize(context.getStaticRemovedFrames().size());
  dynamicEncoder_->encode(context.getDynamicFrames(), context.getDynamicEncodedBytesVector(), parallel);
  staticEncoder_->encode(context.getStaticFrames(), context.getStaticEncodedBytesVector(), parallel);
  staticAddedEncoder_->encode(context.getStaticAddedFrames(), context.getStaticAddedEncodedBytesVector(), parallel);
  staticRemovedEncoder_->encode(context.getStaticRemovedFrames(), context.getStaticRemovedEncodedBytesVector(),
                                parallel);
}
}  // namespace jpcc::encoder