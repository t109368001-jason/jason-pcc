#pragma once

#include <jpcc/common/IJPCCEncoderContext.h>
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

  void convertToCoderType(IJPCCEncoderContext& context, bool parallel);

  void encode(IJPCCEncoderContext& context, bool parallel);
};

}  // namespace jpcc::encoder
