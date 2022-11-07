#include <jpcc/decoder/JPCCDecoderNone.h>

#include <sstream>

namespace jpcc::decoder {

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderNone::decode(std::istream& is, FramePtr& frame) {
  frame = make_shared<Frame>();

  uint32_t length = 0;
  length          = (length << 8) | static_cast<unsigned>(is.get());
  length          = (length << 8) | static_cast<unsigned>(is.get());
  length          = (length << 8) | static_cast<unsigned>(is.get());
  length          = (length << 8) | static_cast<unsigned>(is.get());

  if (!is) return;

  frame->resize(length);
  for (size_t i = 0; i < frame->getPointCount(); i++) {
    PointType& point = (*frame)[i];
    is.read(reinterpret_cast<char*>(&point.x()), sizeof(PointValueType));
    is.read(reinterpret_cast<char*>(&point.y()), sizeof(PointValueType));
    is.read(reinterpret_cast<char*>(&point.z()), sizeof(PointValueType));
  }
}

}  // namespace jpcc::decoder