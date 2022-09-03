#pragma once

#include <jpcc/common/Common.h>

namespace jpcc::coder {

template <typename PointT>
struct JPCCCoderContext {
  FramePtr<PointT>  pclFrame;
  shared_ptr<void>  frame;
  std::vector<char> encodedBytes;
  shared_ptr<void>  reconstructFrame;
  FramePtr<PointT>  reconstructPclFrame;
};

}  // namespace jpcc::coder