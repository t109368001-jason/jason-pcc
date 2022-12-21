#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/encoder/JPCCEncoderParameter.h>

namespace jpcc::encoder {

class JPCCEncoder {
 public:
  using Ptr = shared_ptr<JPCCEncoder>;

 protected:
  JPCCEncoderParameter parameter_;

 public:
  JPCCEncoder(JPCCEncoderParameter parameter);  // NOLINT(google-explicit-constructor)

  virtual bool isConvertToCoderTypeThreadSafe();

  virtual bool isEncodeThreadSafe();

  virtual void convertToCoderType(const FramePtr& frame, std::shared_ptr<void>& coderFrame) = 0;

  virtual void convertToCoderType(const GroupOfFrame&                 frames,
                                  std::vector<std::shared_ptr<void>>& coderFrames,
                                  bool                                parallel);

  virtual void encode(const FramePtr& frame, std::vector<char>& encodedBytes) = 0;

  virtual void encode(const GroupOfFrame& frames, std::vector<std::vector<char>>& encodedBytesVector, bool parallel);
};

}  // namespace jpcc::encoder
