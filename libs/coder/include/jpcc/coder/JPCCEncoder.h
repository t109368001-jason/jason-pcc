#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/coder/JPCCCoderContext.h>
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

  virtual void convertFromPCL(JPCCCoderContext<PointT>& context);

  virtual void encode(JPCCCoderContext<PointT>& context) = 0;
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCEncoder.hpp>
