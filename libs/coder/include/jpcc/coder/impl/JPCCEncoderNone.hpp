#include <sstream>

#include <io_tlv.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCEncoderNone<PointT>::JPCCEncoderNone(const JPCCEncoderParameter& parameter) : JPCCEncoder<PointT>(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCEncoderNone<PointT>::isThreadSafe() {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderNone<PointT>::convertFromPCL(const FramePtr<PointT>& pclFrame, shared_ptr<void>& frame) {
  frame = pclFrame;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderNone<PointT>::encode(const shared_ptr<void>& frame, std::vector<char>& encodedBytes) {}

}  // namespace jpcc::coder