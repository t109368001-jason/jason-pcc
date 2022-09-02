#pragma once

#include <jpcc/coder/JPCCDecoder.h>
#include <jpcc/coder/JPCCDecoderParameter.h>

namespace jpcc::coder {

class JPCCDecoderAdapter {
 public:
  template <typename PointT>
  [[nodiscard]] static typename JPCCDecoder<PointT>::Ptr build(const JPCCDecoderParameter& parameter);
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCDecoderAdapter.hpp>
