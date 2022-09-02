#pragma once

#include <jpcc/common/Common.h>

namespace jpcc::coder {

template <typename PointT>
struct JPCCEncoderContext {
  FramePtr<PointT> pclFrame;
  shared_ptr<void> frame;
  shared_ptr<void> encodedFrame;
};

}  // namespace jpcc::coder