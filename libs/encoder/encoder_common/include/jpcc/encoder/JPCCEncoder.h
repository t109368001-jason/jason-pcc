#pragma once

#include <jpcc/common/Common.h>

namespace jpcc::encoder {

class JPCCEncoder {
 public:
  using Ptr = shared_ptr<JPCCEncoder>;

  using CoderFrame        = void;
  using CoderFramePtr     = std::shared_ptr<CoderFrame>;
  using CoderGroupOfFrame = std::vector<CoderFramePtr>;

 public:
  virtual bool isConvertToCoderTypeThreadSafe();

  virtual bool isEncodeThreadSafe();

  virtual void convertToCoderType(const FramePtr& frame, CoderFramePtr& coderFrame) = 0;

  virtual void convertToCoderType(const GroupOfFrame& frames, CoderGroupOfFrame& coderFrames, bool parallel);

  virtual void encode(const CoderFramePtr& coderFrame, std::ostream& os) = 0;

  virtual void encode(const CoderGroupOfFrame& coderFrames, std::ostream& os, bool parallel);
};

}  // namespace jpcc::encoder
