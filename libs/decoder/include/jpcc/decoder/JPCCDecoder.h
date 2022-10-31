#pragma once

#include <jpcc/common/Common.h>

namespace jpcc::decoder {

template <typename PointT>
class JPCCDecoder {
 public:
  using Ptr = shared_ptr<JPCCDecoder>;

 public:
  virtual bool isConvertToPCLThreadSafe();

  virtual void decode(std::istream& is, shared_ptr<void>& reconstructFrame) = 0;

  virtual void convertToPCL(shared_ptr<void>& reconstructFrame, FramePtr<PointT>& reconstructPclFrame) = 0;

  virtual void convertToPCL(std::vector<shared_ptr<void>>& reconstructFrames,
                            GroupOfFrame<PointT>&          reconstructPclFrames,
                            bool                           parallel);
};

}  // namespace jpcc::decoder

#include <jpcc/decoder/impl/JPCCDecoder.hpp>
