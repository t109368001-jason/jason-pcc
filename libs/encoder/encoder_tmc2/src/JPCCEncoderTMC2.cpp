#include <jpcc/encoder/JPCCEncoderTMC2.h>

#include <sstream>

#include <PCCEncoderParameters.h>
#include <PCCPointSet.h>

using namespace pcc;

namespace jpcc::encoder {

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC2::JPCCEncoderTMC2(const JPCCEncoderTMC2Parameter parameter) : JPCCEncoder(), parameter_(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderTMC2::isConvertToCoderTypeThreadSafe() {
  // TODO
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderTMC2::isEncodeThreadSafe() {
  // TODO
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2::convertToCoderType(const FramePtr& frame, CoderFramePtr& coderFrame) {
  // TODO
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC2::encode(const CoderFramePtr& coderFrame, std::vector<char>& encodedBytes) {
  // TODO
}

}  // namespace jpcc::encoder