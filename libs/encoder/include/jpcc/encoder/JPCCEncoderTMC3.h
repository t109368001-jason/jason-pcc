#pragma once

#include <jpcc/encoder/JPCCEncoder.h>

namespace jpcc::encoder {

class JPCCEncoderTMC3 : public virtual JPCCEncoder {
 public:
  JPCCEncoderTMC3(const JPCCEncoderParameter& parameter);  // NOLINT(google-explicit-constructor)

  bool isConvertToCoderTypeThreadSafe() override;

  bool isEncodeThreadSafe() override;

  void convertToCoderType(const FramePtr& frame, CoderFramePtr& coderFrame) override;

  void encode(const CoderFramePtr& coderFrame, std::vector<char>& encodedBytes) override;
};

}  // namespace jpcc::encoder
