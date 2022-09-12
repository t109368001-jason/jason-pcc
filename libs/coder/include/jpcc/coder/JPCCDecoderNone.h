#pragma once

#include <jpcc/coder/JPCCDecoder.h>

namespace jpcc::coder {

template <typename PointT>
class JPCCDecoderNone : public virtual JPCCDecoder<PointT> {
 public:
  JPCCDecoderNone(const JPCCDecoderParameter& parameter);

  void decode(JPCCContext<PointT>& context) override;

  void convertToPCL(JPCCContext<PointT>& context) override;
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCDecoderNone.hpp>
