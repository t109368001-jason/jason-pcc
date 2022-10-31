#pragma once

#include <jpcc/encoder/JPCCEncoder.h>
#include <jpcc/encoder/JPCCEncoderParameter.h>

namespace jpcc::encoder {

template <typename PointT>
class JPCCEncoderAdapter {
 protected:
  typename JPCCEncoder<PointT>::Ptr dynamicEncoder_;
  typename JPCCEncoder<PointT>::Ptr staticEncoder_;
  typename JPCCEncoder<PointT>::Ptr staticAddedEncoder_;
  typename JPCCEncoder<PointT>::Ptr staticRemovedEncoder_;

 public:
  JPCCEncoderAdapter(const JPCCEncoderParameter& dynamicParameter, const JPCCEncoderParameter& staticParameter);

  void convertFromPCL(IJPCCEncoderContext<PointT>& context, bool parallel);

  void encode(IJPCCEncoderContext<PointT>& context, bool parallel);
};

}  // namespace jpcc::encoder

#include <jpcc/encoder/impl/JPCCEncoderAdapter.hpp>
