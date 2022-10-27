#pragma once

#include <jpcc/coder/JPCCDecoder.h>

namespace jpcc::coder {

template <typename PointT>
class JPCCDecoderNone : public virtual JPCCDecoder<PointT> {
 public:
  bool isConvertToPCLThreadSafe() override;

  void decode(std::istream& is, shared_ptr<void>& reconstructFrame) override;

  void convertToPCL(shared_ptr<void>& reconstructFrame, FramePtr<PointT>& reconstructPclFrame) override;
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCDecoderNone.hpp>
