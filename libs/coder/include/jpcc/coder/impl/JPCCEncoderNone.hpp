#include <sstream>

#include <io_tlv.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCEncoderNone<PointT>::JPCCEncoderNone(const JPCCEncoderParameter& parameter) : JPCCEncoder<PointT>(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderNone<PointT>::convertFromPCL(JPCCCoderContext<PointT>& context) {
  context.frame = context.pclFrame;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCEncoderNone<PointT>::encode(JPCCCoderContext<PointT>& context) {}

}  // namespace jpcc::coder