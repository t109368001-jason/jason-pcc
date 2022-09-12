#pragma once

#include <jpcc/coder/JPCCEncoder.h>

namespace jpcc::coder {

template <typename PointT>
class JPCCEncoderNone : public virtual JPCCEncoder<PointT> {
 public:
  JPCCEncoderNone(const JPCCEncoderParameter& parameter);

  void convertFromPCL(JPCCContext<PointT>& context) override;

  void encode(JPCCContext<PointT>& context) override;
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCEncoderNone.hpp>
