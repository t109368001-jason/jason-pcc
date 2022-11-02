#include <jpcc/decoder/JPCCDecoderNone.h>

#include <sstream>

#include "io_tlv.h"

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
    Frame::PointType& point = (*frame)[i];
    is.read(reinterpret_cast<char*>(&point.x()), sizeof(((Frame::PointType*)nullptr)->x()));
    is.read(reinterpret_cast<char*>(&point.y()), sizeof(((Frame::PointType*)nullptr)->y()));
    is.read(reinterpret_cast<char*>(&point.z()), sizeof(((Frame::PointType*)nullptr)->z()));
  }
}

}  // namespace jpcc::decoder