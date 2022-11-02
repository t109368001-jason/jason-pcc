#pragma once

#include <jpcc/decoder/JPCCDecoder.h>

namespace jpcc::decoder {

class JPCCDecoderNone : public virtual JPCCDecoder {
 public:
  void decode(std::istream& is, FramePtr& frame) override;
};

}  // namespace jpcc::decoder
