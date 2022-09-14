#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/coder/JPCCEncoderParameter.h>

namespace jpcc::coder {

template <typename PointT>
class JPCCEncoder {
 public:
  using Ptr = shared_ptr<JPCCEncoder>;

 protected:
  JPCCEncoderParameter parameter_;

 public:
  JPCCEncoder(const JPCCEncoderParameter& parameter);

  virtual bool isThreadSafe();

  virtual void convertFromPCL(const FramePtr<PointT>& pclFrame, shared_ptr<void>& frame) = 0;

  virtual void convertFromPCL(const GroupOfFrame<PointT>&    pclFrames,
                              std::vector<shared_ptr<void>>& frames,
                              bool                           parallel);

  virtual void encode(const shared_ptr<void>& frame, std::vector<char>& encodedBytes) = 0;

  virtual void encode(const std::vector<shared_ptr<void>>& frames,
                      std::vector<std::vector<char>>&      encodedFramesBytes,
                      bool                                 parallel);
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCEncoder.hpp>
