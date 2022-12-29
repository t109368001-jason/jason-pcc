#include <jpcc/encoder/JPCCEncoderTMC2.h>

#include <sstream>

#include <PCCEncoderParameters.h>
#include <PCCPointSet.h>

using namespace pcc;

namespace jpcc::encoder {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC2::JPCCEncoderTMC2(const JPCCEncoderTMC2Parameter parameter) : JPCCEncoder(), parameter_(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderTMC2::isConvertToCoderTypeThreadSafe() { return true; }

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderTMC2::isEncodeThreadSafe() {
  // TODO
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2::convertToCoderType(const FramePtr& frame, CoderFramePtr& coderFrame) {
  auto _coderFrame = std::make_shared<PCCPointSet3>();
  coderFrame       = _coderFrame;
  _coderFrame->resize(frame->getPointCount());
  for (size_t i = 0; i < _coderFrame->getPointCount(); i++) {
    auto point          = (*frame)[i];
    using TMC2ValueType = std::remove_reference_t<std::remove_const_t<decltype(((PCCPoint3D*)nullptr)->x())>>;
    _coderFrame->setPosition(i, PCCPoint3D((TMC2ValueType)point[0], (TMC2ValueType)point[1], (TMC2ValueType)point[2]));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2::encode(const CoderFramePtr& coderFrame, std::vector<char>& encodedBytes) {
  // TODO
}

}  // namespace jpcc::encoder