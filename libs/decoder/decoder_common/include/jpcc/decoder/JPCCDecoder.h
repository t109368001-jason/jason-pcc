#pragma once

#include <jpcc/common/Common.h>

namespace jpcc::decoder {

class JPCCDecoder {
 public:
  using Ptr = shared_ptr<JPCCDecoder>;

 public:
  virtual bool isConvertFromCoderTypeThreadSafe();

  virtual void decode(std::istream& is, std::shared_ptr<void>& coderReconstructFrame) = 0;

  virtual void convertFromCoderType(const std::shared_ptr<void>& coderFrame, FramePtr& frame) = 0;

  virtual void convertFromCoderType(const std::vector<std::shared_ptr<void>>& coderFrames,
                                    GroupOfFrame&                             frames,
                                    bool                                      parallel);
};

}  // namespace jpcc::decoder
