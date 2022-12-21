#pragma once

#include <jpcc/encoder/JPCCEncoder.h>

namespace jpcc::encoder {

class JPCCEncoderNone : public virtual JPCCEncoder {
 public:
  JPCCEncoderNone(const JPCCEncoderParameter& parameter);  // NOLINT(google-explicit-constructor)

  bool isConvertToCoderTypeThreadSafe() override;

  bool isEncodeThreadSafe() override;

  void convertToCoderType(const FramePtr& frame, shared_ptr<void>& coderFrame) override;

  void encode(const FramePtr& frame, std::vector<char>& encodedBytes) override;
};

}  // namespace jpcc::encoder
