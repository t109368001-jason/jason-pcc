#include <sstream>

#include <io_tlv.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCDecoderNone<PointT>::JPCCDecoderNone(const JPCCDecoderParameter& parameter) : JPCCDecoder<PointT>(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCDecoderNone<PointT>::isThreadSafe() {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderNone<PointT>::decode(const std::vector<char>& encodedBytes, shared_ptr<void>& reconstructFrame) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderNone<PointT>::convertToPCL(shared_ptr<void>& reconstructFrame, FramePtr<PointT>& reconstructPclFrame) {
  reconstructPclFrame = std::static_pointer_cast<Frame<PointT>>(reconstructFrame);
}

}  // namespace jpcc::coder