#pragma once

#include <jpcc/coder/JPCCEncoder.h>
#include <jpcc/coder/JPCCEncoderParameter.h>

namespace jpcc::coder {

class JPCCEncoderAdapter {
 public:
  template <typename PointT>
  [[nodiscard]] static typename JPCCEncoder<PointT>::Ptr build(const JPCCEncoderParameter& parameter);
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCEncoderAdapter.hpp>
