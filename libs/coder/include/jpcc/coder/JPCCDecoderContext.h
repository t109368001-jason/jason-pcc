#pragma once

#include <jpcc/common/Common.h>

namespace jpcc::coder {

template <typename PointT>
struct JPCCDecoderContext {
  FramePtr<PointT> pclFrame;
  shared_ptr<void> frame;
  shared_ptr<void> encodedFrame;
  shared_ptr<void> reconstructFrame;
  FramePtr<PointT> reconstructPclFrame;
};

}  // namespace jpcc::coder