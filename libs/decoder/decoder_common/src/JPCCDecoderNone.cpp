#include <jpcc/decoder/JPCCDecoderNone.h>

#include <sstream>

namespace jpcc::decoder {

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCDecoderNone::isConvertFromCoderTypeThreadSafe() {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderNone::decode(std::istream& is, std::shared_ptr<void>& coderReconstructFrame) {
  FramePtr frame        = make_shared<Frame>();
  coderReconstructFrame = std::static_pointer_cast<void>(frame);

  uint32_t length = 0;
  length          = (length << 8) | static_cast<unsigned>(is.get());
  length          = (length << 8) | static_cast<unsigned>(is.get());
  length          = (length << 8) | static_cast<unsigned>(is.get());
  length          = (length << 8) | static_cast<unsigned>(is.get());

  if (!is) return;

  frame->resize(length);
  for (size_t i = 0; i < frame->getPointCount(); i++) {
    PointType& point = (*frame)[i];
    is.read(reinterpret_cast<char*>(&point[0]), sizeof(PointValueType));
    is.read(reinterpret_cast<char*>(&point[1]), sizeof(PointValueType));
    is.read(reinterpret_cast<char*>(&point[2]), sizeof(PointValueType));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderNone::convertFromCoderType(const std::shared_ptr<void>& coderFrame, FramePtr& frame) {
  frame = std::static_pointer_cast<Frame>(coderFrame);
}

}  // namespace jpcc::decoder