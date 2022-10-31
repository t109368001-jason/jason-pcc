#pragma once

#include <jpcc/decoder/JPCCDecoder.h>
#include <PCCTMC3Decoder.h>

namespace jpcc::decoder {

class PCCTMC3Decoder3LambdaCallbacks : public pcc::PCCTMC3Decoder3::Callbacks {
 protected:
  const std::function<void(const pcc::CloudFrame& frame)>& onOutputCloud_;

 public:
  PCCTMC3Decoder3LambdaCallbacks(const std::function<void(const pcc::CloudFrame& frame)>& onOutputCloud);

 protected:
  void onOutputCloud(const pcc::CloudFrame& frame) override;
};

template <typename PointT>
class JPCCDecoderTMC3 : public virtual JPCCDecoder<PointT> {
 public:
  bool isConvertToPCLThreadSafe() override;

  void decode(std::istream& is, shared_ptr<void>& reconstructFrame) override;

  void convertToPCL(shared_ptr<void>& reconstructFrame, FramePtr<PointT>& reconstructPclFrame) override;
};

}  // namespace jpcc::decoder

#include <jpcc/decoder/impl/JPCCDecoderTMC3.hpp>
