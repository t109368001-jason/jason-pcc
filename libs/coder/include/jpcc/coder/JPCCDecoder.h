#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/common/JPCCContext.h>
#include <jpcc/coder/JPCCDecoderParameter.h>

namespace jpcc::coder {

template <typename PointT>
class JPCCDecoder {
 public:
  using Ptr = shared_ptr<JPCCDecoder>;

 protected:
  JPCCDecoderParameter parameter_;

 public:
  JPCCDecoder(const JPCCDecoderParameter& parameter);

  virtual void decode(JPCCContext<PointT>& context) = 0;

  virtual void convertToPCL(JPCCContext<PointT>& context) = 0;
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCDecoder.hpp>
