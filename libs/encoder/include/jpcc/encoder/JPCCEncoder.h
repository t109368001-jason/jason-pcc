#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/encoder/JPCCEncoderParameter.h>

namespace jpcc::encoder {

class JPCCEncoder {
 public:
  using Ptr = shared_ptr<JPCCEncoder>;

  using CoderFrame        = void;
  using CoderFramePtr     = std::shared_ptr<CoderFrame>;
  using CoderGroupOfFrame = std::vector<CoderFramePtr>;

 protected:
  JPCCEncoderParameter parameter_;

 public:
  JPCCEncoder(JPCCEncoderParameter parameter);  // NOLINT(google-explicit-constructor)

  virtual bool isConvertToCoderTypeThreadSafe();

  virtual bool isEncodeThreadSafe();

  virtual void convertToCoderType(const FramePtr& frame, CoderFramePtr& coderFrame) = 0;

  virtual void convertToCoderType(const GroupOfFrame& frames, CoderGroupOfFrame& coderFrames, bool parallel);

  virtual void encode(const CoderFramePtr& coderFrame, std::vector<char>& encodedBytes) = 0;

  virtual void encode(const CoderGroupOfFrame&        coderFrames,
                      std::vector<std::vector<char>>& encodedBytesVector,
                      bool                            parallel);
};

}  // namespace jpcc::encoder
