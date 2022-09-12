#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/coder/JPCCEncoderParameter.h>

namespace jpcc::coder {

template <typename PointT>
class JPCCEncoder {
 public:
  using Ptr = shared_ptr<JPCCEncoder>;

 protected:
  JPCCEncoderParameter parameter_;

 public:
  JPCCEncoder(const JPCCEncoderParameter& parameter);

  virtual void convertFromPCL(const FramePtr<PointT>& pclFrame, shared_ptr<void>& frame) = 0;

  virtual void encode(const shared_ptr<void>& frame, std::vector<char>& encodedBytes) = 0;
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCEncoder.hpp>
