#pragma once

#include <jpcc/decoder/JPCCDecoder.h>

namespace jpcc::decoder {

template <typename PointT>
class JPCCDecoderNone : public virtual JPCCDecoder<PointT> {
 public:
  bool isConvertToPCLThreadSafe() override;

  void decode(std::istream& is, shared_ptr<void>& reconstructFrame) override;

  void convertToPCL(shared_ptr<void>& reconstructFrame, FramePtr<PointT>& reconstructPclFrame) override;
};

}  // namespace jpcc::decoder

#include <jpcc/decoder/impl/JPCCDecoderNone.hpp>
