#include <jpcc/encoder/JPCCEncoderNone.h>

#include <sstream>

namespace jpcc::encoder {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderNone::JPCCEncoderNone() : JPCCEncoder() {}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderNone::isConvertToCoderTypeThreadSafe() { return true; }

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderNone::isEncodeThreadSafe() { return true; }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderNone::convertToCoderType(const FramePtr& frame, CoderFramePtr& coderFrame) {
  coderFrame = std::static_pointer_cast<CoderFrame>(frame);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderNone::encode(const CoderFramePtr& coderFrame, std::ostream& os) {
#if !defined(NDEBUG)
  auto startPosition = os.tellp();
#endif
  auto _coderFrame = std::static_pointer_cast<Frame>(coderFrame);

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

  assert((_coderFrame->getPointCount() * (sizeof(PointValueType) * 3) + 4) == (os.tellp() - startPosition));
}

}  // namespace jpcc::encoder