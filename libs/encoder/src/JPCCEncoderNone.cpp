#include <jpcc/encoder/JPCCEncoderNone.h>

#include <sstream>

#include "io_tlv.h"

namespace jpcc::encoder {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderNone::JPCCEncoderNone(const JPCCEncoderParameter& parameter) : JPCCEncoder(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderNone::isConvertToCoderTypeThreadSafe() { return true; }

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderNone::isEncodeThreadSafe() { return true; }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderNone::convertToCoderType(const FramePtr& frame, CoderFramePtr& coderFrame) {
  coderFrame = std::static_pointer_cast<CoderFrame>(frame);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderNone::encode(const CoderFramePtr& coderFrame, std::vector<char>& encodedBytes) {
  auto              _coderFrame = std::static_pointer_cast<Frame>(coderFrame);
  std::stringstream os;

  const auto length = uint32_t(_coderFrame->getPointCount());

  os.put(char(length >> 24));
  os.put(char(length >> 16));
  os.put(char(length >> 8));
  os.put(char(length >> 0));

  for (size_t i = 0; i < _coderFrame->getPointCount(); i++) {
    PointType& point = (*_coderFrame)[i];
    os.write(reinterpret_cast<char*>(&point[0]), sizeof(PointValueType));
    os.write(reinterpret_cast<char*>(&point[1]), sizeof(PointValueType));
    os.write(reinterpret_cast<char*>(&point[2]), sizeof(PointValueType));
  }

#if !defined(NDEBUG)
  size_t oldSize = encodedBytes.size();
#endif
  std::string tmpString = os.str();
  for (char& i : tmpString) { encodedBytes.push_back(i); }
  assert((coderFrame->getPointCount() * (sizeof(PointValueType) * 3) + 4) == (encodedBytes.size() - oldSize));
}

}  // namespace jpcc::encoder