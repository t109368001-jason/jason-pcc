#pragma once

#include <jpcc/encoder/JPCCEncoder.h>
#include <PCCTMC3Encoder.h>

namespace jpcc::encoder {

class PCCTMC3Encoder3LambdaCallbacks : public pcc::PCCTMC3Encoder3::Callbacks {
 protected:
  const std::function<void(const pcc::PayloadBuffer& buffer)>& onOutputBuffer_;
  const std::function<void(const pcc::PCCPointSet3& set3)>&    onPostRecolour_;

 public:
  PCCTMC3Encoder3LambdaCallbacks(const std::function<void(const pcc::PayloadBuffer& buffer)>& onOutputBuffer,
                                 const std::function<void(const pcc::PCCPointSet3& set3)>&    onPostRecolour);

 protected:
  void onOutputBuffer(const pcc::PayloadBuffer& buffer) override;
  void onPostRecolour(const pcc::PCCPointSet3& set3) override;
};

class JPCCEncoderTMC3 : public virtual JPCCEncoder {
 public:
  JPCCEncoderTMC3(const JPCCEncoderParameter& parameter);  // NOLINT(google-explicit-constructor)

  bool isEncodeThreadSafe() override;

  void encode(const FramePtr& frame, std::vector<char>& encodedBytes) override;
};

}  // namespace jpcc::encoder
