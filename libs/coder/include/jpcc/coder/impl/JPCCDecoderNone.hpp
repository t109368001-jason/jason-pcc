#include <sstream>

#include <io_tlv.h>

namespace jpcc::coder {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCDecoderNone<PointT>::JPCCDecoderNone(const JPCCDecoderParameter& parameter) : JPCCDecoder<PointT>(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool JPCCDecoderNone<PointT>::isConvertToPCLThreadSafe() {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderNone<PointT>::decode(std::istream& is, shared_ptr<void>& reconstructFrame) {
  reconstructFrame = make_shared<Frame<PointT>>();
  auto _frame     = std::static_pointer_cast<Frame<PointT>>(reconstructFrame);

  uint32_t length = 0;
  length          = (length << 8) | static_cast<unsigned>(is.get());
  length          = (length << 8) | static_cast<unsigned>(is.get());
  length          = (length << 8) | static_cast<unsigned>(is.get());
  length          = (length << 8) | static_cast<unsigned>(is.get());

  if (!is) return;

  _frame->resize(length);
  for (auto& point : _frame->points) {
    is.read(reinterpret_cast<char*>(&point.x), sizeof(((PointT*)0)->x));
    is.read(reinterpret_cast<char*>(&point.y), sizeof(((PointT*)0)->y));
    is.read(reinterpret_cast<char*>(&point.z), sizeof(((PointT*)0)->z));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCDecoderNone<PointT>::convertToPCL(shared_ptr<void>& reconstructFrame, FramePtr<PointT>& reconstructPclFrame) {
  reconstructPclFrame = std::static_pointer_cast<Frame<PointT>>(reconstructFrame);
}

}  // namespace jpcc::coder