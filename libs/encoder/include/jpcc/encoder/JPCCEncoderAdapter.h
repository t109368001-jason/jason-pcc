#pragma once

#include <jpcc/common/JPCCContext.h>
#include <jpcc/encoder/JPCCEncoder.h>
#include <jpcc/encoder/JPCCEncoderParameter.h>

namespace jpcc::encoder {

class JPCCEncoderAdapter {
 protected:
  typename JPCCEncoder::Ptr dynamicEncoder_;
  typename JPCCEncoder::Ptr staticEncoder_;
  typename JPCCEncoder::Ptr staticAddedEncoder_;
  typename JPCCEncoder::Ptr staticRemovedEncoder_;

 public:
  JPCCEncoderAdapter(const JPCCEncoderParameter& dynamicParameter, const JPCCEncoderParameter& staticParameter);

  void convertToCoderType(JPCCContext& context, bool parallel);

  void encode(JPCCContext& context, bool parallel);
};

}  // namespace jpcc::encoder
