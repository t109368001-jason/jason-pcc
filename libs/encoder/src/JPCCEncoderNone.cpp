#include <jpcc/encoder/JPCCEncoderNone.h>

#include <sstream>

#include "io_tlv.h"

namespace jpcc::encoder {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderNone::JPCCEncoderNone(const JPCCEncoderParameter& parameter) : JPCCEncoder(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderNone::encode(const FramePtr& frame, std::vector<char>& encodedBytes) {
  std::stringstream os;

  const auto length = uint32_t(frame->getPointCount());

  os.put(char(length >> 24));
  os.put(char(length >> 16));
  os.put(char(length >> 8));
  os.put(char(length >> 0));

  for (size_t i = 0; i < frame->getPointCount(); i++) {
    Frame::PointType& point = (*frame)[i];
    os.write(reinterpret_cast<char*>(&point.x()), sizeof(((Frame::PointType*)nullptr)->x()));
    os.write(reinterpret_cast<char*>(&point.y()), sizeof(((Frame::PointType*)nullptr)->y()));
    os.write(reinterpret_cast<char*>(&point.z()), sizeof(((Frame::PointType*)nullptr)->z()));
  }

#if !defined(NDEBUG)
  size_t oldSize = encodedBytes.size();
#endif
  std::string tmpString = os.str();
  for (char& i : tmpString) { encodedBytes.push_back(i); }
  assert((frame->getPointCount() * (sizeof(((Frame::PointType*)0)->x()) + sizeof(((Frame::PointType*)0)->y()) +
                                    sizeof(((Frame::PointType*)0)->z())) +
          4) == (encodedBytes.size() - oldSize));
}

}  // namespace jpcc::encoder