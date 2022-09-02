#pragma once

#include <jpcc/encoder/JPCCEncoder.h>
#include <PCCTMC3Encoder.h>

namespace jpcc::encoder {

template <typename PointT>
class JPCCEncoderTMC3 : public virtual JPCCEncoder<PointT>, protected pcc::PCCTMC3Encoder3::Callbacks {
 protected:
  pcc::PCCTMC3Encoder3 encoder;

 public:
  JPCCEncoderTMC3(const JPCCEncoderParameter& parameter);

  void convertFromPCL(JPCCEncoderContext<PointT>& context) override;

  void encode(JPCCEncoderContext<PointT>& context) override;

 protected:
  void onOutputBuffer(const pcc::PayloadBuffer& buffer) override;
  void onPostRecolour(const pcc::PCCPointSet3& set3) override;
};

}  // namespace jpcc::encoder

#include <jpcc/encoder/impl/JPCCEncoderTMC3.hpp>
