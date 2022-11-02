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
  JPCCEncoder(JPCCEncoderParameter parameter);

  virtual bool isEncodeThreadSafe();

  virtual void encode(const FramePtr& frame, std::vector<char>& encodedBytes) = 0;

  virtual void encode(const GroupOfFrame& frames, std::vector<std::vector<char>>& encodedBytesVector, bool parallel);
};

}  // namespace jpcc::encoder
