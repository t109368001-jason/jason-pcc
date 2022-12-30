#pragma once

#include <jpcc/common/IJPCCDecoderContext.h>
#include <jpcc/common/JPCCContext.h>
#include <jpcc/decoder/JPCCDecoder.h>

namespace jpcc::decoder {

class JPCCDecoderAdapter {
 protected:
  typename JPCCDecoder::Ptr dynamicDecoder_;
  typename JPCCDecoder::Ptr staticDecoder_;
  typename JPCCDecoder::Ptr staticAddedDecoder_;
  typename JPCCDecoder::Ptr staticRemovedDecoder_;

 public:
  void set(const JPCCContext& context);

  void decode(JPCCContext& context, size_t frameCount);

  void convertFromCoderType(JPCCContext& context, bool parallel);
};

}  // namespace jpcc::decoder
