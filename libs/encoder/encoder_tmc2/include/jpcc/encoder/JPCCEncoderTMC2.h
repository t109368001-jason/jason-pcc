#pragma once

#include <jpcc/encoder/JPCCEncoderNone.h>
#include <jpcc/encoder/JPCCEncoderTMC2Parameter.h>

namespace jpcc::encoder {

class JPCCEncoderTMC2 : public virtual JPCCEncoder {
 protected:
  JPCCEncoderTMC2Parameter parameter_;

 public:
  JPCCEncoderTMC2(JPCCEncoderTMC2Parameter parameter);  // NOLINT(google-explicit-constructor)

  bool isConvertToCoderTypeThreadSafe() override;

  bool isEncodeThreadSafe() override;

  void convertToCoderType(const FramePtr& frame, CoderFramePtr& coderFrame) override;

  void encode(const CoderFramePtr& coderFrame, std::vector<char>& encodedBytes) override;
};

}  // namespace jpcc::encoder
