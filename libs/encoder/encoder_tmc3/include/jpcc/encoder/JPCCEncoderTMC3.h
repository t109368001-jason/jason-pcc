#pragma once

#include <jpcc/encoder/JPCCEncoder.h>
#include <jpcc/encoder/JPCCEncoderTMC3Parameter.h>

namespace jpcc::encoder {

class JPCCEncoderTMC3 : public virtual JPCCEncoder {
 protected:
  JPCCEncoderTMC3Parameter parameter_;

 public:
  JPCCEncoderTMC3(JPCCEncoderTMC3Parameter parameter);  // NOLINT(google-explicit-constructor)

  bool isConvertToCoderTypeThreadSafe() override;

  bool isEncodeThreadSafe() override;

  void convertToCoderType(const FramePtr& frame, CoderFramePtr& coderFrame) override;

  void encode(const CoderFramePtr& coderFrame, std::ostream& os) override;
};

}  // namespace jpcc::encoder
