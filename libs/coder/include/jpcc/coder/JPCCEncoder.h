#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/common/JPCCContext.h>
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

  virtual void convertFromPCL(JPCCContext<PointT>& context) = 0;

  virtual void encode(JPCCContext<PointT>& context) = 0;
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCEncoder.hpp>
