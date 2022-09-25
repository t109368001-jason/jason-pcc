#pragma once

#include <jpcc/common/Common.h>
#include <jpcc/coder/JPCCDecoderParameter.h>

namespace jpcc::coder {

template <typename PointT>
class JPCCDecoder {
 public:
  using Ptr = shared_ptr<JPCCDecoder>;

 protected:
  JPCCDecoderParameter parameter_;

 public:
  JPCCDecoder(JPCCDecoderParameter parameter);

  virtual bool isDecodeThreadSafe();

  virtual bool isConvertToPCLThreadSafe();

  virtual void decode(std::istream& is, shared_ptr<void>& reconstructFrame) = 0;

  virtual void decode(std::istream& is, std::vector<shared_ptr<void>>& reconstructFrames, bool parallel);

  virtual void convertToPCL(shared_ptr<void>& reconstructFrame, FramePtr<PointT>& reconstructPclFrame) = 0;

  virtual void convertToPCL(std::vector<shared_ptr<void>>& reconstructFrames,
                            GroupOfFrame<PointT>&          reconstructPclFrames,
                            bool                           parallel);
};

}  // namespace jpcc::coder

#include <jpcc/coder/impl/JPCCDecoder.hpp>
