#pragma once

#include <jpcc/encoder/JPCCEncoder.h>

namespace jpcc::encoder {

class JPCCEncoderTMC3 : public virtual JPCCEncoder {
 public:
  JPCCEncoderTMC3(const JPCCEncoderParameter& parameter);  // NOLINT(google-explicit-constructor)

  bool isEncodeThreadSafe() override;

  void encode(const FramePtr& frame, std::vector<char>& encodedBytes) override;
};

}  // namespace jpcc::encoder
