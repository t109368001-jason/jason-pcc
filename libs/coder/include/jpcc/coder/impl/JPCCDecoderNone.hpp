#include <sstream>

#include <io_tlv.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCDecoderNone<PointT>::JPCCDecoderNone(const JPCCDecoderParameter& parameter) : JPCCDecoder<PointT>(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderNone<PointT>::decode(JPCCContext<PointT>& context) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderNone<PointT>::convertToPCL(JPCCContext<PointT>& context) {
  context.reconstructPclFrame = std::static_pointer_cast<Frame<PointT>>(context.reconstructFrame);
}

}  // namespace jpcc::coder