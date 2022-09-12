#pragma once

#include <jpcc/coder/JPCCEncoder.h>
#include <PCCTMC3Encoder.h>

namespace jpcc::coder {

template <typename PointT>
class JPCCEncoderTMC3 : public virtual JPCCEncoder<PointT>, protected pcc::PCCTMC3Encoder3::Callbacks {
 protected:
  pcc::PCCTMC3Encoder3 encoder_;
  std::vector<char>*   encodedBytesPtr_;

 public:
  JPCCEncoderTMC3(const JPCCEncoderParameter& parameter);

  void convertFromPCL(const FramePtr<PointT>& pclFrame, shared_ptr<void>& frame) override;

  void encode(const shared_ptr<void>& frame, std::vector<char>& encodedBytes) override;

 protected:
  void onOutputBuffer(const pcc::PayloadBuffer& buffer) override;
  void onPostRecolour(const pcc::PCCPointSet3& set3) override;
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCEncoderTMC3.hpp>
