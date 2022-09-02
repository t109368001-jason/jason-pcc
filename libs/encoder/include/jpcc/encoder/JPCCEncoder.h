#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/encoder/JPCCEncoderParameter.h>

namespace jpcc::encoder {

template <typename PointT>
struct JPCCEncoderContext {
  FramePtr<PointT> pclFrame;
  shared_ptr<void> frame;
  shared_ptr<void> encodedFrame;
};

template <typename PointT>
class JPCCEncoder {
 public:
  using Ptr = shared_ptr<JPCCEncoder>;

 protected:
  JPCCEncoderParameter parameter_;

 public:
  JPCCEncoder(const JPCCEncoderParameter& parameter);

  virtual void convertFromPCL(JPCCEncoderContext<PointT>& context);

  virtual void encode(JPCCEncoderContext<PointT>& context) = 0;
};

}  // namespace jpcc::encoder

#include <jpcc/encoder/impl/JPCCEncoder.hpp>
