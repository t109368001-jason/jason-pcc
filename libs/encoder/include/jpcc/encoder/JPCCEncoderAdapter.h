#pragma once

#include <jpcc/encoder/JPCCEncoder.h>
#include <jpcc/encoder/JPCCEncoderParameter.h>

namespace jpcc::encoder {

class JPCCEncoderAdapter {
 public:
  template <typename PointT>
  [[nodiscard]] static typename JPCCEncoder<PointT>::Ptr build(const JPCCEncoderParameter& parameter);
};

}  // namespace jpcc::encoder

#include <jpcc/encoder/impl/JPCCEncoderAdapter.hpp>
