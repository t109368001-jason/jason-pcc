#pragma once

#include <jpcc/common/IJPCCDecoderContext.h>
#include <jpcc/common/JPCCHeader.h>
#include <jpcc/decoder/JPCCDecoder.h>

namespace jpcc::decoder {

class JPCCDecoderAdapter {
 protected:
  typename JPCCDecoder::Ptr dynamicDecoder_;
  typename JPCCDecoder::Ptr staticDecoder_;
  typename JPCCDecoder::Ptr staticAddedDecoder_;
  typename JPCCDecoder::Ptr staticRemovedDecoder_;

 public:
  void set(JPCCHeader header);

  void decode(std::istream& is, IJPCCDecoderContext& context, size_t frameCount);

  void convertFromCoderType(IJPCCDecoderContext& context, bool parallel);
};

}  // namespace jpcc::decoder
