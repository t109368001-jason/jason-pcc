#pragma once

#include <jpcc/coder/JPCCDecoder.h>

namespace jpcc::coder {

template <typename PointT>
class JPCCDecoderNone : public virtual JPCCDecoder<PointT> {
 public:
  JPCCDecoderNone(const JPCCDecoderParameter& parameter);

  void decode(const std::vector<char>& encodedBytes, shared_ptr<void>& reconstructFrame) override;

  void convertToPCL(shared_ptr<void>& reconstructFrame, FramePtr<PointT>& reconstructPclFrame) override;
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCDecoderNone.hpp>
