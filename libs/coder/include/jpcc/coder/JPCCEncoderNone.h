#pragma once

#include <jpcc/coder/JPCCEncoder.h>

namespace jpcc::coder {

template <typename PointT>
class JPCCEncoderNone : public virtual JPCCEncoder<PointT> {
 public:
  JPCCEncoderNone(const JPCCEncoderParameter& parameter);

  bool isConvertFromPCLThreadSafe() override;

  void convertFromPCL(const FramePtr<PointT>& pclFrame, shared_ptr<void>& frame) override;

  void encode(const shared_ptr<void>& frame, std::vector<char>& encodedBytes) override;
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCEncoderNone.hpp>
