#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/coder/JPCCCoderContext.h>
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

  virtual void decode(JPCCCoderContext<PointT>& context) = 0;

  virtual void convertToPCL(JPCCCoderContext<PointT>& context) = 0;
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCDecoder.hpp>
