#pragma once

#include <jpcc/decoder/JPCCDecoder.h>

namespace jpcc::decoder {

class JPCCDecoderTMC3 : public virtual JPCCDecoder {
 public:
  bool isConvertFromCoderTypeThreadSafe() override;

  void decode(std::istream& is, std::shared_ptr<void>& coderReconstructFrame) override;

  void convertFromCoderType(const std::shared_ptr<void>& coderFrame, FramePtr& frame) override;
};

}  // namespace jpcc::decoder
