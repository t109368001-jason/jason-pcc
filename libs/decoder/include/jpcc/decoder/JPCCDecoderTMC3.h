#pragma once

#include <jpcc/decoder/JPCCDecoder.h>
#include <PCCTMC3Decoder.h>

namespace jpcc::decoder {

class PCCTMC3Decoder3LambdaCallbacks : public pcc::PCCTMC3Decoder3::Callbacks {
 protected:
  const std::function<void(const pcc::CloudFrame& frame)>& onOutputCloud_;

 public:
  PCCTMC3Decoder3LambdaCallbacks(  // NOLINT(google-explicit-constructor)
      const std::function<void(const pcc::CloudFrame& frame)>& onOutputCloud);

 protected:
  void onOutputCloud(const pcc::CloudFrame& frame) override;
};

class JPCCDecoderTMC3 : public virtual JPCCDecoder {
 public:
  void decode(std::istream& is, FramePtr& frame) override;
};

}  // namespace jpcc::decoder
