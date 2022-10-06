#pragma once

#include <jpcc/coder/JPCCEncoder.h>
#include <jpcc/coder/JPCCEncoderParameter.h>

namespace jpcc::coder {

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

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCEncoderAdapter.hpp>
