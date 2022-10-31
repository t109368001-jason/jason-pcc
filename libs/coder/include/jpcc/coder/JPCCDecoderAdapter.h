#pragma once

#include <jpcc/common/JPCCHeader.h>
#include <jpcc/coder/JPCCDecoder.h>

namespace jpcc::coder {

template <typename PointT>
class JPCCDecoderAdapter {
 protected:
  typename JPCCDecoder<PointT>::Ptr dynamicDecoder_;
  typename JPCCDecoder<PointT>::Ptr staticDecoder_;
  typename JPCCDecoder<PointT>::Ptr staticAddedDecoder_;
  typename JPCCDecoder<PointT>::Ptr staticRemovedDecoder_;

 public:
  void set(JPCCHeader header);

  void decode(std::istream& is, IJPCCDecoderContext<PointT>& context, size_t frameCount, bool parallel);

  void convertToPCL(IJPCCDecoderContext<PointT>& context, bool parallel);
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCDecoderAdapter.hpp>
