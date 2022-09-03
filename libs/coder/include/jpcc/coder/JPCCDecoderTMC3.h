#pragma once

#include <jpcc/coder/JPCCDecoder.h>
#include <PCCTMC3Decoder.h>

namespace jpcc::coder {

template <typename PointT>
class JPCCDecoderTMC3 : public virtual JPCCDecoder<PointT>, protected pcc::PCCTMC3Decoder3::Callbacks {
 protected:
  pcc::PCCTMC3Decoder3      decoder;
  JPCCCoderContext<PointT>* contextPtr_;

 public:
  JPCCDecoderTMC3(const JPCCDecoderParameter& parameter);

  void decode(JPCCCoderContext<PointT>& context) override;

  void convertToPCL(JPCCCoderContext<PointT>& context) override;

 protected:
  void onOutputCloud(const pcc::CloudFrame& frame) override;
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCDecoderTMC3.hpp>
