#include <jpcc/decoder/JPCCDecoderTMC2.h>

#include <sstream>

#include <PCCPointSet.h>

using namespace pcc;

namespace jpcc::decoder {

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCDecoderTMC2::isConvertFromCoderTypeThreadSafe() {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderTMC2::decode(std::istream& is, std::shared_ptr<void>& coderReconstructFrame) {
  // TODO
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCDecoderTMC2::convertFromCoderType(const std::shared_ptr<void>& coderFrame, FramePtr& frame) {
  auto _coderFrame = std::static_pointer_cast<PCCPointSet3>(coderFrame);
  frame->resize(_coderFrame->getPointCount());
  for (size_t i = 0; i < frame->getPointCount(); i++) {
    const auto& coderPoint = (*_coderFrame)[i];
    auto&       point      = (*frame)[i];
    point[0]               = coderPoint.x();
    point[1]               = coderPoint.y();
    point[2]               = coderPoint.z();
  }
}

}  // namespace jpcc::decoder