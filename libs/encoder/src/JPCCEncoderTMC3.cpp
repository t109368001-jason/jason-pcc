#include <jpcc/encoder/JPCCEncoderTMC3.h>

#include <sstream>

#include "io_tlv.h"

namespace jpcc::encoder {

//////////////////////////////////////////////////////////////////////////////////////////////
PCCTMC3Encoder3LambdaCallbacks::PCCTMC3Encoder3LambdaCallbacks(
    const std::function<void(const pcc::PayloadBuffer& buffer)>& onOutputBuffer,
    const std::function<void(const pcc::PCCPointSet3& set3)>&    onPostRecolour) :
    onOutputBuffer_(onOutputBuffer), onPostRecolour_(onPostRecolour) {}

//////////////////////////////////////////////////////////////////////////////////////////////
void PCCTMC3Encoder3LambdaCallbacks::onOutputBuffer(const pcc::PayloadBuffer& buffer) { onOutputBuffer_(buffer); }

//////////////////////////////////////////////////////////////////////////////////////////////
void PCCTMC3Encoder3LambdaCallbacks::onPostRecolour(const pcc::PCCPointSet3& set3) { onPostRecolour_(set3); }

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC3::JPCCEncoderTMC3(const JPCCEncoderParameter& parameter) : JPCCEncoder(parameter) {}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderTMC3::isEncodeThreadSafe() { return true; }

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC3::encode(const FramePtr& frame, std::vector<char>& encodedBytes) {
  std::function<void(const pcc::PayloadBuffer& buffer)> onOutputBuffer =
      [&encodedBytes](const pcc::PayloadBuffer& buffer) {
        std::stringstream os;
        pcc::writeTlv(buffer, os);
#if !defined(NDEBUG)
        size_t oldSize = encodedBytes.size();
#endif
        std::string tmpString = os.str();
        for (char& i : tmpString) { encodedBytes.push_back(i); }
        assert((buffer.size() + 5) == (encodedBytes.size() - oldSize));
      };
  std::function<void(const pcc::PCCPointSet3& set3)> onPostRecolour = [](const pcc::PCCPointSet3& set3) {};

  if (frame->getPointCount() == 0) {
    pcc::PayloadBuffer buffer;
    buffer.type = pcc::PayloadType::kSequenceParameterSet;

    onOutputBuffer(buffer);
    return;
  }

  JPCCEncoderTMC3Parameter       param = this->parameter_.tmc3;
  PCCTMC3Encoder3LambdaCallbacks callback(onOutputBuffer, onPostRecolour);
  pcc::PCCTMC3Encoder3           encoder;
  encoder.compress(*frame, &param, &callback, nullptr);
  std::cout << __FUNCTION__ << "() "
            << "bytes=" << encodedBytes.size() << std::endl;
}

}  // namespace jpcc::encoder