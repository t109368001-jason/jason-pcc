#pragma once

#include <jpcc/encoder/JPCCEncoder.h>

namespace jpcc::encoder {

class JPCCEncoderNone : public virtual JPCCEncoder {
 public:
  JPCCEncoderNone(const JPCCEncoderParameter& parameter);

  void encode(const FramePtr& frame, std::vector<char>& encodedBytes) override;
};

}  // namespace jpcc::encoder
