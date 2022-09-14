#pragma once

#include <jpcc/coder/JPCCEncoder.h>
#include <PCCTMC3Encoder.h>

namespace jpcc::coder {

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

template <typename PointT>
class JPCCEncoderTMC3 : public virtual JPCCEncoder<PointT> {
 public:
  JPCCEncoderTMC3(const JPCCEncoderParameter& parameter);

  void convertFromPCL(const FramePtr<PointT>& pclFrame, shared_ptr<void>& frame) override;

  void encode(const shared_ptr<void>& frame, std::vector<char>& encodedBytes) override;
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCEncoderTMC3.hpp>
