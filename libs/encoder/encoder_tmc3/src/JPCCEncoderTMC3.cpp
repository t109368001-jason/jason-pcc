#include <jpcc/encoder/JPCCEncoderTMC3.h>

#include <sstream>

#include <boost/log/trivial.hpp>

#include <io_tlv.h>
#include <PCCTMC3Encoder.h>

using namespace pcc_tmc3;

namespace jpcc::encoder {

class PCCTMC3Encoder3LambdaCallbacks : public PCCTMC3Encoder3::Callbacks {
 protected:
  const std::function<void(const PayloadBuffer& buffer)>& onOutputBuffer_;
  const std::function<void(const PCCPointSet3& set3)>&    onPostRecolour_;

 public:
  PCCTMC3Encoder3LambdaCallbacks(const std::function<void(const PayloadBuffer& buffer)>& onOutputBuffer,
                                 const std::function<void(const PCCPointSet3& set3)>&    onPostRecolour);

 protected:
  void onOutputBuffer(const PayloadBuffer& buffer) override;
  void onPostRecolour(const PCCPointSet3& set3) override;
};

//////////////////////////////////////////////////////////////////////////////////////////////
PCCTMC3Encoder3LambdaCallbacks::PCCTMC3Encoder3LambdaCallbacks(
    const std::function<void(const PayloadBuffer& buffer)>& onOutputBuffer,
    const std::function<void(const PCCPointSet3& set3)>&    onPostRecolour) :
    onOutputBuffer_(onOutputBuffer), onPostRecolour_(onPostRecolour) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PCCTMC3Encoder3LambdaCallbacks::onOutputBuffer(const PayloadBuffer& buffer) {
  onOutputBuffer_(buffer);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PCCTMC3Encoder3LambdaCallbacks::onPostRecolour(const PCCPointSet3& set3) {
  onPostRecolour_(set3);
}

//////////////////////////////////////////////////////////////////////////////////////////////
JPCCEncoderTMC3::JPCCEncoderTMC3(const JPCCEncoderTMC3Parameter parameter) : JPCCEncoder(), parameter_(parameter) {
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderTMC3::isConvertToCoderTypeThreadSafe() {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool JPCCEncoderTMC3::isEncodeThreadSafe() {
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC3::convertToCoderType(const FramePtr& frame, CoderFramePtr& coderFrame) {
  coderFrame       = std::make_shared<PCCPointSet3>();
  auto _coderFrame = std::static_pointer_cast<PCCPointSet3>(coderFrame);
  _coderFrame->resize(frame->getPointCount());
  for (size_t i = 0; i < _coderFrame->getPointCount(); i++) {
    (*_coderFrame)[i].x() = (*frame)[i][0];
    (*_coderFrame)[i].y() = (*frame)[i][1];
    (*_coderFrame)[i].z() = (*frame)[i][2];
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCEncoderTMC3::encode(const CoderFramePtr& coderFrame, std::ostream& os) {
  auto                                             startPosition  = os.tellp();
  std::function<void(const PayloadBuffer& buffer)> onOutputBuffer = [&os](const PayloadBuffer& buffer) {
#if !defined(NDEBUG)
    auto _startPosition = os.tellp();
#endif
    writeTlv(buffer, os);
    assert((buffer.size() + 5) == (os.tellp() - _startPosition));
  };
  std::function<void(const PCCPointSet3& set3)> onPostRecolour = [](const PCCPointSet3& set3) {};

  PCCPointSet3 ff;
  auto         _coderFrame = std::static_pointer_cast<PCCPointSet3>(coderFrame);
  if (_coderFrame->getPointCount() == 0) {
    PayloadBuffer buffer;
    buffer.type = PayloadType::kSequenceParameterSet;

    onOutputBuffer(buffer);
    return;
  }

  EncoderParams                  param = *std::static_pointer_cast<EncoderParams>(this->parameter_.params_);
  PCCTMC3Encoder3LambdaCallbacks callback(onOutputBuffer, onPostRecolour);
  PCCTMC3Encoder3                encoder;

  encoder.compress(*_coderFrame, &param, &callback, nullptr);
  BOOST_LOG_TRIVIAL(info) << "bytes=" << (os.tellp() - startPosition);
}

}  // namespace jpcc::encoder