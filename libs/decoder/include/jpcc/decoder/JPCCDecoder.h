#pragma once

#include <jpcc/common/Common.h>

namespace jpcc::decoder {

class JPCCDecoder {
 public:
  using Ptr = shared_ptr<JPCCDecoder>;

 public:
  virtual bool isConvertToPCLThreadSafe();

  virtual void decode(std::istream& is, FramePtr& reconstructFrame) = 0;
};

}  // namespace jpcc::decoder
